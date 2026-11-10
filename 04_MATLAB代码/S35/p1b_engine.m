function varargout=p1b_engine(operation,varargin)
% Every mutable simulation/planning/observer state belongs to C, no persistent.
switch operation
    case 'init',varargout{1}=initialize(varargin{:});
    case 'step',[varargout{1},varargout{2}]=advance(varargin{:});
    otherwise,error('p1b:Operation','Unknown operation');
end
end
function C=initialize(caseDef,cfg,s)
d=p1b_resolve_case(caseDef,s.protocol);steps=round(d.duration_s/.05);delay=round(d.delay_s/.05);
old=rng;cleanup=onCleanup(@()rng(old));rng(d.seed,'twister');noise=randn(steps+4,4);rngState=rng; %#ok<NASGU>
x=zeros(12,1);x(7:8)=d.initial_ne(:);x(12)=deg2rad(d.initial_heading_deg);flow=p1b_current(d.current_schedule,0);
x(1)=d.initial_ur_mps+flow(1)*cos(x(12))+flow(2)*sin(x(12));x(2)=-flow(1)*sin(x(12))+flow(2)*cos(x(12));
C=struct('schema_version','p1b-complete-state-v1','case_definition',d,'next_tick',0,'x',x,'n',zeros(2,1), ...
    'controller',[],'rx',[],'supervisor',[],'policy_state',[], ...
    'planner_states',struct(),'planner_state_epochs',struct(),'pending_jobs',{{}},'global_epoch',0,'last_policy_epoch',-1, ...
    'sensor_ring',nan(delay+1,9),'sensor_objects',{{}},'sensor_sample_count',0,'detected_static_ids',{{}}, ...
    'external_noise_tape',noise,'external_noise_index',1,'rng_state',rngState, ...
    'delivery_latency_tape',nan(ceil(steps/10)+4,1),'timing_mode','record','timing_tape_index',1, ...
    'collision_pending',false,'goal_pending',false,'finished',false,'termination','running', ...
    'last_public',struct(),'last_reference',struct(),'delivered_epochs',zeros(1,0), ...
    'frozen_protocol_sha256',p1b_hash(fullfile(fileparts(mfilename('fullpath')),'protocol.json')), ...
    'frozen_cases_sha256',p1b_hash(fullfile(fileparts(mfilename('fullpath')),'cases.json')));
C.sensor_objects=cell(delay+1,1);
C.physics_contract=struct('mp',25,'rp',[.05;0;-.35],'V_c',0,'beta_c',0,'n_min',cfg.n_min_radps,'n_max',cfg.n_max_radps);
end
function [C,R]=advance(C,cfg,s)
assert(~C.finished);h=.05;t=C.next_tick*h;tick=C.next_tick;dcase=C.case_definition;scene=dcase.scene;
R=struct('tick',tick,'t_s',t,'state_before',stateRow(C,t),'state_after',[], ...
    'action',struct(),'observation',struct(),'supervisor',struct(),'receiver_events',[], ...
    'job_bundles',{{}},'received',{{}},'handoff',struct(),'prediction',struct(),'trace',[], ...
    'geometry',struct(),'public_objects',[],'recovery_job',[],'selection_forecasts',{{}});
if C.collision_pending,C.finished=true;C.termination='collision';return;end
if C.goal_pending,C.finished=true;C.termination='arrived';return;end
if tick>=round(dcase.duration_s/h)
    C.finished=true;C.termination='completed';if strcmp(dcase.expected_termination,'arrived'),C.termination='timeout';end;return;
end

flow=p1b_current(dcase.current_schedule,t);C.physics_contract.V_c=norm(flow);C.physics_contract.beta_c=atan2(flow(2),flow(1));
noise=C.external_noise_tape(C.external_noise_index,:)*dcase.noise_scale;C.external_noise_index=C.external_noise_index+1;
sample=[C.x(1)+cfg.speed_noise_std_mps*noise(1),C.x(2)+.03*noise(2), ...
    atan2(sin(C.x(12)+deg2rad(cfg.heading_noise_std_deg)*noise(3)),cos(C.x(12)+deg2rad(cfg.heading_noise_std_deg)*noise(3))), ...
    C.x(6)+cfg.yaw_rate_noise_std_radps*noise(4),C.x(7:8)',t,flow];
[newObjects,C.detected_static_ids]=p1b_obstacles(scene,t,C.x(7:8)',C.detected_static_ids);
C.sensor_sample_count=C.sensor_sample_count+1;wi=mod(C.sensor_sample_count-1,size(C.sensor_ring,1))+1;
C.sensor_ring(wi,:)=sample;C.sensor_objects{wi}=newObjects;
source=max(1,C.sensor_sample_count-round(dcase.delay_s/h));ri=mod(source-1,size(C.sensor_ring,1))+1;
sample=C.sensor_ring(ri,:);objects=C.sensor_objects{ri};age=t-sample(7);measuredFlow=sample(8:9);
ur=sample(1)-measuredFlow(1)*cos(sample(3))-measuredFlow(2)*sin(sample(3));
vr=sample(2)+measuredFlow(1)*sin(sample(3))-measuredFlow(2)*cos(sample(3));
estimate=struct('ur_mps',ur,'psi_rad',sample(3),'r_radps',sample(4));
groundNE=[sample(1)*cos(sample(3))-sample(2)*sin(sample(3)),sample(1)*sin(sample(3))+sample(2)*cos(sample(3))];
psi=sample(3)+age*sample(4);
public=struct('t_s',t,'measurement_t_s',sample(7),'measurement_ne',sample(5:6), ...
    'ne',sample(5:6)+age*groundNE,'psi_rad',psi,'r_radps',sample(4),'ur_mps',ur,'vr_mps',vr, ...
    'current_estimate_ne_mps',measuredFlow,'ground_u_mps',ur+measuredFlow(1)*cos(psi)+measuredFlow(2)*sin(psi), ...
    'ground_v_mps',vr-measuredFlow(1)*sin(psi)+measuredFlow(2)*cos(psi), ...
    'control_estimate',estimate,'n_measured_radps',C.n','stop_heading_rad',deg2rad(dcase.initial_heading_deg), ...
    'previous_speed_mps',0,'previous_heading_rad',deg2rad(dcase.initial_heading_deg));
if ~isempty(C.rx),public.stop_heading_rad=C.rx.shaped_heading_rad;public.previous_heading_rad=C.rx.shaped_heading_rad;end
if ~isempty(C.controller),public.previous_speed_mps=C.controller.speed_reference_mps;end
R.observation=struct('tick',tick,'t_s',t,'measurement_t_s',sample(7),'measurement_age_s',age, ...
    'ground_u_estimate_mps',sample(1),'ground_v_estimate_mps',sample(2),'psi_estimate_rad',sample(3),'r_estimate_radps',sample(4), ...
    'current_estimate_north_mps',measuredFlow(1),'current_estimate_east_mps',measuredFlow(2), ...
    'ur_estimate_mps',ur,'vr_estimate_mps',vr,'sog_estimate_mps',hypot(sample(1),sample(2)), ...
    'measurement_north_m',sample(5),'measurement_east_m',sample(6), ...
    'projected_north_m',public.ne(1),'projected_east_m',public.ne(2),'projected_psi_rad',psi, ...
    'shaft_measured_left_radps',C.n(1),'shaft_measured_right_radps',C.n(2), ...
    'noise_tape_index',C.external_noise_index-1,'buffer_count',C.sensor_sample_count,'buffer_source_index',ri,'observed_object_count',numel(objects));
R.public_objects=objects;
map=struct('bounds_ne',scene.bounds_ne,'goal_ne',scene.goal_ne,'global_route_ne',scene.global_route_ne, ...
    'lookahead_m',scene.lookahead_m,'footprint_radius_m',scene.footprint_radius_m,'safety_margin_m',scene.safety_margin_m);

% Completed epoch results become visible only at their shared ready event.
incoming={};keep={};deliveredFailure=false;
for k=1:numel(C.pending_jobs)
    bundle=C.pending_jobs{k};
    if bundle.ready_time<=t+1e-10
        packet=bundle.selected_packet;incoming{end+1}=packet;C.delivered_epochs(end+1)=bundle.epoch; %#ok<AGROW>
        eligible=causalEligible(packet,C.rx,t,s);
        R.received{end+1}=struct('arrival_time',t,'scheduled_arrival_time',bundle.ready_time,'epoch',bundle.epoch, ...
            'policy',dcase.strategy,'packet',packet,'causally_eligible',eligible,'selection_log',bundle.selection_log); %#ok<AGROW>
        if eligible
            if strcmp(packet.status,'failed'),deliveredFailure=true;end
            if bundle.epoch>C.last_policy_epoch && ~isempty(bundle.policy_proposal.method)
                proposal=bundle.policy_proposal;
                if isempty(C.policy_state) || ~strcmp(C.policy_state.method,proposal.method),proposal.last_switch_time_s=t;end
                C.policy_state=proposal;C.last_policy_epoch=bundle.epoch;
            end
        end
        for j=1:numel(bundle.jobs)
            job=bundle.jobs{j};key=matlab.lang.makeValidName(job.method);
            if ~isfield(C.planner_state_epochs,key) || bundle.epoch>C.planner_state_epochs.(key)
                C.planner_states.(key)=job.decision.planner_state;C.planner_state_epochs.(key)=bundle.epoch;
            end
        end
    else,keep{end+1}=bundle;end %#ok<AGROW>
end
C.pending_jobs=keep;

candidateCpu=0;selectionCpu=0;
if mod(tick,round(scene.decision_dt_s/h))==0
    C.global_epoch=C.global_epoch+1;epoch=C.global_epoch;methods=requestMethods(dcase);
    % S04: shared causal execution context; never the full simulation state.
    executionSettings=struct();
    executionSettingNames={'control_dt_s','heading_rate_limit_radps','speed_max_mps', ...
        'compute_budget_s','max_arrival_age_s','initial_heading_rad','pending_capacity', ...
        'reference_ttl_s','reference_horizon_s','node_dt_s'};
    for contextField=1:numel(executionSettingNames)
        executionSettings.(executionSettingNames{contextField})=s.(executionSettingNames{contextField});
    end
    visibleMode='TRACK';if ~isempty(C.supervisor),visibleMode=C.supervisor.mode;end
    ctx=struct('public',public,'controller',C.controller,'rx',C.rx,'incoming',{incoming}, ...
        'mode',visibleMode,'cfg',cfg,'settings',executionSettings,'epoch',epoch);
    jobs=cell(1,numel(methods));packets=cell(1,numel(methods));
    for k=1:numel(methods)
        previous=struct('target_speed_mps',public.previous_speed_mps,'target_heading_rad',public.previous_heading_rad,'planner_state',struct());
        key=matlab.lang.makeValidName(methods{k});if isfield(C.planner_states,key),previous.planner_state=C.planner_states.(key);end
        [packets{k},jobs{k}]=p1b_candidate(methods{k},public,objects,scene,previous,epoch,t,s,ctx);
        candidateCpu=candidateCpu+jobs{k}.actual_cpu_s;
    end
    timer=tic;[chosen,proposal,selection,forecasts]=p1b_select(dcase.strategy,jobs,packets,public,objects,map,C.controller,C.rx,C.policy_state,C.supervisor,cfg,s);selectionCpu=toc(timer);
    if chosen>0,packet=packets{chosen};else
        packet=packets{1};packet.planner_id='NONE';packet.reference_id=sprintf('P1B-NO-CHOICE-%06d',epoch);packet.status='failed';packet.failure_reason='selector_no_safe_candidate';
    end
    latency=max(candidateCpu+selectionCpu,dcase.injected_latency_s);
    if strcmp(C.timing_mode,'replay'),assert(isfinite(C.delivery_latency_tape(epoch)));latency=C.delivery_latency_tape(epoch);
    else,C.delivery_latency_tape(epoch)=latency;end
    packet.ready_time=t+latency;C.timing_tape_index=epoch+1;
    for k=1:numel(packets),packets{k}.ready_time=packet.ready_time;jobs{k}.shared_ready_time=packet.ready_time;end
    selection.actual_cpu_s=selectionCpu;selection.total_candidate_cpu_s=candidateCpu;
    selection.generated_from_time=t;selection.ready_time=packet.ready_time;
    bundle=struct('epoch',epoch,'generated_from_time',t,'ready_time',packet.ready_time, ...
        'jobs',{jobs},'candidate_packets',{packets},'selected_packet',packet,'policy_proposal',proposal,'selection_log',selection);
    C.pending_jobs{end+1}=bundle;R.job_bundles={bundle};R.selection_forecasts=forecasts;
end

preview=C.rx;if ~isempty(preview),preview.supervisor_latched=false;end
[normalRef,~]=p0_reference_receiver(t,incoming,preview,s,struct('trigger',false,'clear_authorized',true,'reason','public_preview'));
timer=tic;[nextSup,override,guard,prediction,recoveryJob]=p1b_supervisor(t,public,objects,map,normalRef,C.controller,C.supervisor,cfg,s,deliveredFailure,dcase.recovery_enabled);guardCpu=toc(timer);
before=snapshot(C);
sup=struct('trigger',override.enabled,'clear_authorized',~override.enabled,'reason',nextSup.reason);
timer=tic;[reference,C.rx,events]=p0_reference_receiver(t,incoming,C.rx,s,sup);receiverCpu=toc(timer);
plannerReference=reference;
if strcmp(override.mode,'RECOVER')
    % Explicit NEW RECOVER state: continuous actual heading inheritance.
    C.rx.shaped_heading_rad=override.heading_rad;
    reference.heading_rad=override.heading_rad;reference.raw_heading_rad=override.raw_heading_rad;
    reference.speed_mps=override.speed_mps;reference.raw_speed_mps=override.speed_mps;
end
after=snapshot(C);R.handoff=struct('tick',tick,'t_s',t,'before',before,'after',after,'max_difference',0,'exact_preserved',isequaln(before,after));
assert(R.handoff.exact_preserved,'p1b:ExecutionReset','Reference management reset execution state');
C.supervisor=nextSup;C.last_public=public;C.last_reference=reference;
timer=tic;[command,nextController,action]=p0_speed_controller(estimate,struct('speed_mps',reference.speed_mps,'heading_rad',reference.heading_rad),C.controller,cfg);controlCpu=toc(timer);
action.raw_controller_z_u_next=nextController.z_u;action.raw_controller_z_psi_next=nextController.z_psi;
if override.freeze_integrals
    if isempty(C.controller),nextController.z_u=0;nextController.z_psi=0;else,nextController.z_u=C.controller.z_u;nextController.z_psi=C.controller.z_psi;end
end
action.z_u_next=nextController.z_u;action.z_psi_next=nextController.z_psi;
assert(isequal(command.n_cmd_radps(:)',[guard.expected_first_left_radps,guard.expected_first_right_radps]),'p1b:FirstCommand','Actual command differs from common preview');
action.tick=tick;action.t_s=t;action.dt_s=h;action.interval_completed=1;action.override_integrals_frozen=double(override.freeze_integrals);
action.active_reference_id=string(reference.reference_id);action.active_planner_id=string(reference.planner_id);action.reference_sequence=reference.sequence;
action.reference_generated_time_s=reference.generated_from_time;action.reference_ready_time_s=reference.ready_time;action.reference_valid_until_s=reference.valid_until;
action.reference_fresh=double(reference.fresh);action.fallback=double(reference.fallback);action.supervisor_latched=double(override.enabled);
action.execution_source=string(override.execution_source);action.supervisor_mode=string(override.mode);
action.planner_raw_speed_mps=plannerReference.raw_speed_mps;action.planner_raw_heading_rad=plannerReference.raw_heading_rad;
action.raw_reference_speed_mps=reference.raw_speed_mps;action.raw_reference_heading_rad=reference.raw_heading_rad;action.shaped_reference_heading_rad=reference.heading_rad;
action.selected_method=string('');action.policy_switch_count=0;
if ~isempty(C.policy_state),action.selected_method=string(C.policy_state.method);action.policy_switch_count=C.policy_state.switch_count;end
action.candidate_cpu_s=candidateCpu;action.selection_cpu_s=selectionCpu;action.supervisor_cpu_s=guardCpu;action.receiver_cpu_s=receiverCpu;action.control_time_s=controlCpu;
action.recovery_search_cpu_s=guard.recovery_search_cpu_s;
action.total_measured_compute_s=candidateCpu+selectionCpu+guardCpu+receiverCpu+controlCpu;action.control_deadline_exceeded=double(action.total_measured_compute_s>h);
action.n_actual_left_radps=C.n(1);action.n_actual_right_radps=C.n(2);action.current_north_mps=flow(1);action.current_east_mps=flow(2);
action.controller_speed_before_mps=0;action.controller_psi_d_before_rad=estimate.psi_rad;action.controller_r_d_before_radps=0;
if ~isempty(C.controller),action.controller_speed_before_mps=C.controller.speed_reference_mps;action.controller_psi_d_before_rad=C.controller.psi_d_rad;action.controller_r_d_before_radps=C.controller.r_d_radps;end
[nextX,nextN,trace,difference]=p003_dynamics_trace(C.x,C.n,command.n_cmd_radps,C.physics_contract);
world=p1b_world_at(scene,t);g=p003_sweep_interval(trace(:,7:8),t,h/4,world);
shafts=zeros(5,2);water=zeros(5,2);
for k=0:4
    shafts(k+1,:)=(command.n_cmd_radps+(C.n-command.n_cmd_radps)*exp(-k*h/4/.1))';psiK=trace(k+1,12);
    water(k+1,:)=[trace(k+1,1)-flow(1)*cos(psiK)-flow(2)*sin(psiK),trace(k+1,2)+flow(1)*sin(psiK)-flow(2)*cos(psiK)];
end
R.trace=[repmat(tick,5,1),(0:4)',t+(0:4)'*h/4,trace,shafts,repmat(flow,5,1),water];
g.tick=tick;g.t_s=t;g.dynamics_max_difference=difference;R.geometry=g;
action.dynamics_max_difference=difference;action.swept_clearance_m=g.min_clearance_m;action.margin_violation_s=g.margin_violation_s;action.physical_collision=double(g.collision);
R.action=action;R.supervisor=guard;R.receiver_events=events;R.prediction=prediction;R.recovery_job=recoveryJob;
C.x=nextX;C.n=nextN;C.controller=nextController;C.next_tick=tick+1;
C.collision_pending=g.collision;C.goal_pending=norm(nextX(7:8)'-scene.goal_ne)<=scene.goal_radius_m;R.state_after=stateRow(C,t+h);
end
function methods=requestMethods(d)
if d.equal_shadow || any(strcmp(d.strategy,{'RULE','JUDGE'})),methods=d.registry_methods;return;end
switch d.strategy
    case 'FDWA',methods={'L1'};
    case 'FVO',methods={'L2'};
    case 'FASTAR',methods={'ASTAR'};
    otherwise,error('p1b:Policy','Unknown policy');
end
end
function yes=causalEligible(p,rx,t,s)
seq=-1;generation=-Inf;if ~isempty(rx),seq=rx.sequence_watermark;generation=rx.generation_watermark;end
yes=p.sequence>seq && p.generated_from_time>=generation && p.ready_time<=t+1e-10 && t<p.valid_until && ...
    p.ready_time-p.generated_from_time<=s.compute_budget_s+1e-10 && t-p.generated_from_time<=s.max_arrival_age_s+1e-10;
end
function value=snapshot(C)
value=struct('controller',C.controller,'x',C.x','n',C.n','sensor_ring',C.sensor_ring,'sensor_objects',{C.sensor_objects},'sensor_sample_count',C.sensor_sample_count);
end
function row=stateRow(C,t)
flow=p1b_current(C.case_definition.current_schedule,t);xdot=otter_q_BOTH(C.x,C.n,25,[.05;0;-.35],norm(flow),atan2(flow(2),flow(1)));
ur=C.x(1)-flow(1)*cos(C.x(12))-flow(2)*sin(C.x(12));vr=C.x(2)+flow(1)*sin(C.x(12))-flow(2)*cos(C.x(12));
row=[C.next_tick,t,C.x',C.n',ur,vr,hypot(xdot(7),xdot(8)),flow,xdot(7:8)'];
end
