function [gate,dispatch]=s32_pretarget(C,cfg,profileCfg,s)
% Pure S32 projection of the actual checkpoint before a planned job.
% No world/observer/planner/receiver/controller/model calls and no RNG or I/O.
d=C.case_definition; scene=d.scene; tick=C.next_tick; t=tick*.05;
assert(isa(tick,'double') && isscalar(tick) && isfinite(tick) && tick>=0 && ...
 mod(tick,10)==0 && tick<1800 && C.global_epoch==tick/10 && ~d.equal_shadow, ...
 's32:PretargetClock');
if strcmp(d.strategy,'FDWA'),method='L1';
elseif strcmp(d.strategy,'FVO'),method='L2';
else,error('s32:PretargetPolicy','Unexpected fixed policy %s',d.strategy);end
assert(isequaln(cfg,profileCfg) && cfg.control_dt_s==.05 && s.control_dt_s==.05 && ...
 scene.control_dt_s==.05 && scene.decision_dt_s==.5 && all(d.current_schedule(:,2:3)==0,'all'), ...
 's32:PretargetConfiguration');
mode='TRACK';
if ~isempty(C.supervisor)
 assert(isstruct(C.supervisor) && isscalar(C.supervisor) && isfield(C.supervisor,'mode') && ...
  ischar(C.supervisor.mode) && any(strcmp(C.supervisor.mode,{'TRACK','BRAKE','RECOVER'})), ...
  's32:PretargetSupervisor');
 mode=C.supervisor.mode;
end
gate=struct('supported',false,'reason','','mode',mode,'dispatcher_method',method, ...
 'policy',d.strategy,'tick',tick,'t_s',t,'mode_source','actual checkpoint C.supervisor.mode (initial empty means TRACK)');
dispatch=[];
if ~strcmp(mode,'TRACK')
 gate.reason='NON_TRACK_COMMITTED_ORIGIN';return;
end
validateHistory(C,method,tick);

assert(finiteDouble(C.x,[12,1]) && finiteDouble(C.n,[2,1]) && C.sensor_sample_count==tick && ...
 C.external_noise_index==tick+1 && isa(C.sensor_ring,'double') && isequal(size(C.sensor_ring),[3,9]) && ...
 iscell(C.sensor_objects) && isequal(size(C.sensor_objects),[3,1]) && ...
 d.delay_s==.1 && d.noise_scale==1 && isempty(C.detected_static_ids) && ...
 finiteDouble(C.external_noise_tape(C.external_noise_index,:),[1,4]), ...
 's32:PretargetSensors');
noise=C.external_noise_tape(C.external_noise_index,:)*d.noise_scale;
newSample=[C.x(1)+cfg.speed_noise_std_mps*noise(1),C.x(2)+.03*noise(2), ...
 atan2(sin(C.x(12)+deg2rad(cfg.heading_noise_std_deg)*noise(3)), ...
 cos(C.x(12)+deg2rad(cfg.heading_noise_std_deg)*noise(3))), ...
 C.x(6)+cfg.yaw_rate_noise_std_radps*noise(4),C.x(7:8)',t,0,0];
newObjects=observedCircle(scene,t,C.x(7:8)');
count=C.sensor_sample_count+1;
writeIndex=mod(count-1,size(C.sensor_ring,1))+1;
source=max(1,count-round(d.delay_s/.05));
readIndex=mod(source-1,size(C.sensor_ring,1))+1;
if readIndex==writeIndex
 sample=newSample;objects=newObjects;
else
 sample=C.sensor_ring(readIndex,:);objects=C.sensor_objects{readIndex};
end
assert(finiteDouble(sample,[1,9]) && sample(7)==(source-1)*.05 && ...
 all(sample(8:9)==0),'s32:PretargetObservationClock');
% Historical positions are measured without position noise. An observed ship
% is therefore tested at its own stored time and the recorded ego centre.
assert(isequaln(objects,observedCircle(scene,sample(7),sample(5:6))), ...
 's32:PretargetDelayedObjectPayload');
age=t-sample(7);flow=sample(8:9);
ur=sample(1)-flow(1)*cos(sample(3))-flow(2)*sin(sample(3));
vr=sample(2)+flow(1)*sin(sample(3))-flow(2)*cos(sample(3));
estimate=struct('ur_mps',ur,'psi_rad',sample(3),'r_radps',sample(4));
ground=[sample(1)*cos(sample(3))-sample(2)*sin(sample(3)), ...
 sample(1)*sin(sample(3))+sample(2)*cos(sample(3))];
psi=sample(3)+age*sample(4);
public=struct('t_s',t,'measurement_t_s',sample(7),'measurement_ne',sample(5:6), ...
 'ne',sample(5:6)+age*ground,'psi_rad',psi,'r_radps',sample(4),'ur_mps',ur,'vr_mps',vr, ...
 'current_estimate_ne_mps',flow,'ground_u_mps',ur+flow(1)*cos(psi)+flow(2)*sin(psi), ...
 'ground_v_mps',vr-flow(1)*sin(psi)+flow(2)*cos(psi),'control_estimate',estimate, ...
 'n_measured_radps',C.n','stop_heading_rad',deg2rad(d.initial_heading_deg), ...
 'previous_speed_mps',0,'previous_heading_rad',deg2rad(d.initial_heading_deg));
if ~isempty(C.rx)
 public.stop_heading_rad=C.rx.shaped_heading_rad;
 public.previous_heading_rad=C.rx.shaped_heading_rad;
end
if ~isempty(C.controller),public.previous_speed_mps=C.controller.speed_reference_mps;end

incoming={};priorStates=C.planner_states;priorEpochs=C.planner_state_epochs;
for k=1:numel(C.pending_jobs)
 bundle=C.pending_jobs{k};
 if bundle.ready_time<=t+1e-10
  incoming{end+1}=bundle.selected_packet; %#ok<AGROW>
  job=bundle.jobs{1};
  if ~isfield(priorEpochs,method) || bundle.epoch>priorEpochs.(method)
   priorStates.(method)=job.decision.planner_state;priorEpochs.(method)=bundle.epoch;
  end
 end
end
executionSettings=struct();
settingsNames={'control_dt_s','heading_rate_limit_radps','speed_max_mps','compute_budget_s', ...
 'max_arrival_age_s','initial_heading_rad','pending_capacity','reference_ttl_s','reference_horizon_s','node_dt_s'};
for k=1:numel(settingsNames)
 key=settingsNames{k};executionSettings.(key)=s.(key);
end
epoch=C.global_epoch+1;
ctx=struct('public',public,'controller',C.controller,'rx',C.rx,'incoming',{incoming}, ...
 'mode',mode,'cfg',cfg,'settings',executionSettings,'epoch',epoch);
previous=struct('target_speed_mps',public.previous_speed_mps,'target_heading_rad',public.previous_heading_rad, ...
 'planner_state',struct());
if isfield(priorStates,method),previous.planner_state=priorStates.(method);end
sceneNames={'id','bounds_ne','start_ne','goal_ne','global_route_ne','footprint_radius_m','safety_margin_m', ...
 'sensor_range_m','lookahead_m','target_speed_mps','maximum_speed_mps','decision_dt_s','control_dt_s'};
candidateScene=struct();
for k=1:numel(sceneNames)
 key=sceneNames{k};candidateScene.(key)=scene.(key);
end
candidateScene.current_estimate_ne_mps=public.current_estimate_ne_mps;
assert(finiteDouble([public.t_s,public.psi_rad,public.ur_mps,public.r_radps, ...
 previous.target_speed_mps,previous.target_heading_rad],[1,6]) && finiteDouble(public.ne,[1,2]), ...
 's32:PretargetPublicAndHistory');
if strcmp(method,'L1')
 [l1,~]=s21_preflight(ctx,cfg,profileCfg);
 assert(l1.supported,'s32:UnexpectedL1Support','%s',l1.reason);
end
dispatch=struct('method',method,'ctx',ctx,'objects',objects,'previous',previous, ...
 'candidate_scene',candidateScene,'measurement_sample',sample,'new_sample',newSample, ...
 'new_objects',newObjects,'write_index',writeIndex,'read_index',readIndex,'measurement_source_count',source);
gate.supported=true;
end

function objects=observedCircle(scene,t,ego)
% Same exact field order and empty MATLAB struct type as p1b_obstacles.
objects=struct('type',{},'id',{},'bounds_ne',{},'initial_ne',{},'velocity_ne_mps',{}, ...
 'radius_m',{},'observed_at_s',{});
assert(isequal(size(scene.known_rectangles_ne),[0,4]) && isa(scene.known_rectangles_ne,'double') && ...
 isequal(size(scene.hidden_rectangles_ne),[0,4]) && isa(scene.hidden_rectangles_ne,'double') && ...
 isempty(scene.known_ids) && isempty(scene.hidden_ids) && scene.sensor_range_m==35 && ...
 isequal(scene.global_route_ne,[0,0;60,0]) && scene.lookahead_m==9.6,'s32:PretargetScene');
ship=scene.moving_ships;
assert(isstruct(ship) && isscalar(ship) && strcmp(ship.id,'MOVING_01') && ...
 ship.spawn_time_s==0 && isequal(ship.initial_ne,[30,28]) && isequal(ship.velocity_schedule,[0,0,-1]) && ...
 ship.radius_m==2.5 && ~isfield(ship,'despawn_time_s'),'s32:PretargetTrueShip');
center=ship.initial_ne;
velocity=ship.velocity_schedule(2:3);
if t>0,center=center+t*velocity;end
if norm(ego-center)-ship.radius_m<=scene.sensor_range_m
 objects(1)=struct('type','circle','id',ship.id,'bounds_ne',zeros(1,4), ...
  'initial_ne',center,'velocity_ne_mps',velocity,'radius_m',ship.radius_m,'observed_at_s',t);
end
end

function validateHistory(C,method,tick)
if isempty(C.controller)
 assert(tick==0 && finiteDouble(C.controller,[0,0]),'s32:PretargetInitialController');
else
 keys={'speed_reference_mps';'psi_d_rad';'r_d_radps';'z_u';'z_psi'};
 assert(isstruct(C.controller) && isscalar(C.controller) && isequal(fieldnames(C.controller),keys), ...
  's32:PretargetControllerSchema');
 for k=1:numel(keys),assert(finiteDouble(C.controller.(keys{k}),[1,1]),'s32:PretargetControllerValue');end
end
if isempty(C.rx)
 assert(tick==0 && finiteDouble(C.rx,[0,0]),'s32:PretargetInitialReceiver');
else
 r=C.rx;
 keys={'active','pending','sequence_watermark','generation_watermark','shaped_heading_rad','last_time_s', ...
  'supervisor_latched','supervisor_reason','latch_started_at_s'};
 assert(isstruct(r) && isscalar(r) && all(isfield(r,keys)) && iscell(r.pending) && ...
  finiteDouble(r.shaped_heading_rad,[1,1]) && r.last_time_s==(tick-1)*.05 && ...
  islogical(r.supervisor_latched) && isscalar(r.supervisor_latched),'s32:PretargetReceiverSchema');
 if isempty(r.active)
  assert(isa(r.active,'double') && isempty(r.active) && ...
   (r.sequence_watermark==-1 || r.sequence_watermark>=1),'s32:PretargetInactiveReceiver');
 else
  a=r.active;
  assert(isstruct(a) && isscalar(a) && strcmp(a.planner_id,method) && a.sequence==r.sequence_watermark && ...
   a.generated_from_time==r.generation_watermark && isequal(size(a.t_ref),[31,1]) && ...
   finiteDouble(a.t_ref,[31,1]) && finiteDouble(a.ur_ref,[31,1]) && finiteDouble(a.psi_ref,[31,1]) && ...
   finiteDouble(a.psi_unwrapped,[31,1]),'s32:PretargetActiveReceiver');
 end
 for k=1:numel(r.pending)
  p=r.pending{k};
  assert(isstruct(p) && isscalar(p) && strcmp(p.planner_id,method) && ...
   finiteDouble(p.generated_from_time,[1,1]) && finiteDouble(p.ready_time,[1,1]) && ...
   finiteDouble(p.valid_until,[1,1]),'s32:PretargetReceiverPending');
 end
end
p=C.policy_state;
if ~isempty(p)
 assert(isstruct(p) && isscalar(p) && strcmp(p.method,method) && p.switch_count==0 && ...
  finiteDouble(p.last_switch_time_s,[1,1]) && C.last_policy_epoch>=1, 's32:PretargetPolicyHistory');
else
 assert(finiteDouble(p,[0,0]) && C.last_policy_epoch==-1,'s32:PretargetEmptyPolicyHistory');
end
assert(isstruct(C.planner_states) && isscalar(C.planner_states) && isstruct(C.planner_state_epochs) && ...
 isscalar(C.planner_state_epochs) && isequal(fieldnames(C.planner_states),fieldnames(C.planner_state_epochs)) && ...
 all(strcmp(fieldnames(C.planner_states),method)) && iscell(C.pending_jobs), 's32:PretargetPlannerHistory');
if isfield(C.planner_states,method)
 state=C.planner_states.(method);epoch=C.planner_state_epochs.(method);
 assert(isstruct(state) && isscalar(state) && isempty(fieldnames(state)) && ...
  finiteDouble(epoch,[1,1]) && epoch>=1 && epoch<=C.global_epoch && epoch==floor(epoch), ...
  's32:PretargetPlannerHistoryValue');
end
for k=1:numel(C.pending_jobs)
 b=C.pending_jobs{k};
 assert(isstruct(b) && isscalar(b) && iscell(b.jobs) && isequal(size(b.jobs),[1,1]) && ...
  strcmp(b.jobs{1}.method,method) && strcmp(b.selected_packet.planner_id,method) && ...
  finiteDouble(b.ready_time,[1,1]) && finiteDouble(b.epoch,[1,1]) && ...
  b.epoch>=1 && b.epoch<=C.global_epoch,'s32:PretargetPendingHistory');
end
end

function yes=finiteDouble(v,sz)
yes=isa(v,'double') && isreal(v) && isequal(size(v),sz) && all(isfinite(v(:)));
end
