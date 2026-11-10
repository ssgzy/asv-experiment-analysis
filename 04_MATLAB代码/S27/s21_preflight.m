function [check,ctx]=s21_preflight(origin,cfg,profileCfg)
% Pure pretarget support gate copied from dispatcher s10PlannerSupport.
% If given C, reconstruct public observation arithmetically without mutation.
% No RNG, model, planner, controller, receiver, I/O or helper with side effects.
if isfield(origin,'public'),ctx=origin;
else
 C=origin;d=C.case_definition;t=C.next_tick*.05;
 assert(strcmp(d.strategy,'FDWA') && ~d.equal_shadow && all(d.current_schedule(:,2:3)==0,'all'),'s21:SharedConfiguration');
 noise=C.external_noise_tape(C.external_noise_index,:)*d.noise_scale;
 sample=[C.x(1)+cfg.speed_noise_std_mps*noise(1),C.x(2)+.03*noise(2), ...
  atan2(sin(C.x(12)+deg2rad(cfg.heading_noise_std_deg)*noise(3)),cos(C.x(12)+deg2rad(cfg.heading_noise_std_deg)*noise(3))), ...
  C.x(6)+cfg.yaw_rate_noise_std_radps*noise(4),C.x(7:8)',t,0,0];
 ring=C.sensor_ring;count=C.sensor_sample_count+1;wi=mod(count-1,size(ring,1))+1;ring(wi,:)=sample;
 source=max(1,count-round(d.delay_s/.05));ri=mod(source-1,size(ring,1))+1;sample=ring(ri,:);age=t-sample(7);flow=sample(8:9);
 ur=sample(1)-flow(1)*cos(sample(3))-flow(2)*sin(sample(3));vr=sample(2)+flow(1)*sin(sample(3))-flow(2)*cos(sample(3));
 estimate=struct('ur_mps',ur,'psi_rad',sample(3),'r_radps',sample(4));
 ground=[sample(1)*cos(sample(3))-sample(2)*sin(sample(3)),sample(1)*sin(sample(3))+sample(2)*cos(sample(3))];psi=sample(3)+age*sample(4);
 public=struct('t_s',t,'measurement_t_s',sample(7),'measurement_ne',sample(5:6),'ne',sample(5:6)+age*ground, ...
  'psi_rad',psi,'r_radps',sample(4),'ur_mps',ur,'vr_mps',vr,'current_estimate_ne_mps',flow, ...
  'ground_u_mps',ur+flow(1)*cos(psi)+flow(2)*sin(psi),'ground_v_mps',vr-flow(1)*sin(psi)+flow(2)*cos(psi), ...
  'control_estimate',estimate,'n_measured_radps',C.n','stop_heading_rad',deg2rad(d.initial_heading_deg), ...
  'previous_speed_mps',0,'previous_heading_rad',deg2rad(d.initial_heading_deg));
 if ~isempty(C.rx),public.stop_heading_rad=C.rx.shaped_heading_rad;public.previous_heading_rad=C.rx.shaped_heading_rad;end
 if ~isempty(C.controller),public.previous_speed_mps=C.controller.speed_reference_mps;end
 mode='TRACK';if ~isempty(C.supervisor),mode=C.supervisor.mode;end
 ctx=struct('public',public,'controller',C.controller,'rx',C.rx,'mode',mode,'cfg',cfg);
end
check=struct('supported',false,'reason','','mode','MISSING','dispatcher_method','L1','support_source','s10PlannerSupport exact predicates');
if isstruct(ctx) && isscalar(ctx) && isfield(ctx,'mode')
 check.mode=char(ctx.mode);
 if ~any(strcmp(ctx.mode,{'','TRACK'})),check.reason='NON_TRACK_COMMITTED_ORIGIN';return;end
end
if isempty(profileCfg) || ~isstruct(ctx) || ~isscalar(ctx) || ~isfield(ctx,'cfg') || ~isequaln(ctx.cfg,profileCfg)
 check.reason='CONFIG_NOT_IN_BATCH_PROFILE';return;
end
if ~isfield(ctx,'controller'),check.reason='CONTROLLER_SCHEMA_NOT_IN_BATCH_PROFILE';return;end
if ~isempty(ctx.controller)
 expected={'speed_reference_mps';'psi_d_rad';'r_d_radps';'z_u';'z_psi'};
 if ~isstruct(ctx.controller) || ~isscalar(ctx.controller) || ~isequal(sort(fieldnames(ctx.controller)),sort(expected))
  check.reason='CONTROLLER_SCHEMA_NOT_IN_BATCH_PROFILE';return;
 end
 for k=1:numel(expected)
  if ~finiteDouble(ctx.controller.(expected{k}),1),check.reason='CONTROLLER_STATE_NOT_IN_BATCH_PROFILE';return;end
 end
end
if ~isfield(ctx,'public') || ~isstruct(ctx.public) || ~isscalar(ctx.public),check.reason='PUBLIC_STATE_NOT_IN_BATCH_PROFILE';return;end
for key={'t_s','ur_mps','vr_mps','psi_rad','r_radps'}
 if ~isfield(ctx.public,key{1}) || ~finiteDouble(ctx.public.(key{1}),1),check.reason='PUBLIC_STATE_NOT_IN_BATCH_PROFILE';return;end
end
for key={'ne','n_measured_radps','current_estimate_ne_mps'}
 if ~isfield(ctx.public,key{1}) || ~finiteDouble(ctx.public.(key{1}),2),check.reason='PUBLIC_STATE_NOT_IN_BATCH_PROFILE';return;end
end
if ~isfield(ctx.public,'control_estimate') || ~isstruct(ctx.public.control_estimate) || ~isscalar(ctx.public.control_estimate)
 check.reason='CONTROL_ESTIMATE_NOT_IN_BATCH_PROFILE';return;
end
for key={'ur_mps','psi_rad','r_radps'}
 if ~isfield(ctx.public.control_estimate,key{1}) || ~finiteDouble(ctx.public.control_estimate.(key{1}),1),check.reason='CONTROL_ESTIMATE_NOT_IN_BATCH_PROFILE';return;end
end
check.supported=true;
end
function yes=finiteDouble(x,n)
yes=isa(x,'double') && isreal(x) && isvector(x) && numel(x)==n && all(isfinite(x(:)));
end
