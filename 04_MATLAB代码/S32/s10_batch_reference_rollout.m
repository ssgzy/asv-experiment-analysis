function [rolls,stats]=s10_batch_reference_rollout(cores,ctx)
%S10_BATCH_REFERENCE_ROLLOUT Independent N-column nominal forecasts only.
% The S13 private receiver keeps S06 state/event logic and compact output.
% Only controller/shaft/body arithmetic is evaluated in candidate-wide arrays.
% No actual-events or real execution path calls this uniquely named function.
assert(iscell(cores) && isrow(cores) && ~isempty(cores), ...
 's10:CoreList','Expected a nonempty 1-by-N cell array of canonical cores.');
N=numel(cores);h=ctx.cfg.control_dt_s;assert(h==.05);
horizon=12;count=round(horizon/h);assert(abs(count*h-horizon)<1e-12);
assert(any(strcmp(ctx.mode,{'','TRACK'})),'s04:UnsupportedOriginMode','Only TRACK origin is qualified here');
p=ctx.public;cfg=ctx.cfg;t0=p.t_s;
for c=1:N,assert(abs(cores{c}.generated_from_time-t0)<1e-12);end
states=zeros(count+1,9,N);references=zeros(count,12,N);
rx=cell(1,N);eventLogs=cell(1,N);rolls=cell(1,N);firstCommands=zeros(2,N);
initialRow=[p.t_s,reshape(p.ne,1,2),p.ur_mps,p.vr_mps,p.psi_rad,p.r_radps,reshape(p.n_measured_radps,1,2)];
for c=1:N
 rx{c}=ctx.rx;eventLogs{c}={};states(1,:,c)=initialRow;
end
% The caller's support gate admits scalar body/controller values; no coercion.
u=repmat(p.ur_mps,1,N);v=repmat(p.vr_mps,1,N);
psi=repmat(p.psi_rad,1,N);r=repmat(p.r_radps,1,N);
ne=repmat(reshape(p.ne,2,1),1,N);
n=repmat(p.n_measured_radps(:),1,N);
flow=reshape(p.current_estimate_ne_mps,1,2);
controller=[];
if ~isempty(ctx.controller)
 controller=struct('speed_reference_mps',repmat(ctx.controller.speed_reference_mps,1,N), ...
  'psi_d_rad',repmat(ctx.controller.psi_d_rad,1,N), ...
  'r_d_radps',repmat(ctx.controller.r_d_radps,1,N), ...
  'z_u',repmat(ctx.controller.z_u,1,N),'z_psi',repmat(ctx.controller.z_psi,1,N));
end
syntheticIndex=0;receiverCalls=0;controllerBatches=0;bodyBatches=0;
supervisor=struct('trigger',false,'clear_authorized',true,'reason','s04_conditional_track');
% These expressions are unchanged S08 constants, evaluated once for all columns.
midDecay=exp(-h/(2*.1));endDecay=exp(-h/.1);
for k=1:count
 t=t0+(k-1)*h;
 dueGenerations=[];dueSequences=[];
 while t0+syntheticIndex*.5+.05<=t+1e-10
  tg=t0+syntheticIndex*.5;
  dueGenerations(end+1)=tg;dueSequences(end+1)=ctx.epoch+syntheticIndex; %#ok<AGROW>
  syntheticIndex=syntheticIndex+1;
 end
 rawSpeed=zeros(1,N);rawHeading=zeros(1,N);referenceSpeed=zeros(1,N);referenceHeading=zeros(1,N);
 fresh=false(1,N);fallback=false(1,N);sequence=zeros(1,N);
 estimateUr=zeros(1,N);estimatePsi=zeros(1,N);estimateR=zeros(1,N);
 for c=1:N
  incoming={};if k==1,incoming=ctx.incoming;end
  for j=1:numel(dueGenerations)
   incoming{end+1}=syntheticPacket(cores{c},dueGenerations(j),dueSequences(j)); %#ok<AGROW>
  end
  [reference,rx{c},ev]=s13_forecast_receiver(t,incoming,rx{c},ctx.settings,supervisor);
  receiverCalls=receiverCalls+1;
  if ~isempty(ev)
   log=eventLogs{c};log{end+1}=struct('t_s',t,'events',ev);eventLogs{c}=log; %#ok<AGROW>
  end
  if k==1,estimate=ctx.public.control_estimate;
  else,estimate=struct('ur_mps',u(c),'psi_rad',psi(c),'r_radps',r(c));end
  % Retain each scalar validation before filling double arrays. In particular,
  % a legacy non-double reference must not be silently converted and accepted.
  for name={'ur_mps','psi_rad','r_radps'}
   validateattributes(estimate.(name{1}),{'double'},{'real','finite','scalar'});
  end
  for name={'speed_mps','heading_rad'}
   validateattributes(reference.(name{1}),{'double'},{'real','finite','scalar'});
  end
  estimateUr(c)=estimate.ur_mps;estimatePsi(c)=estimate.psi_rad;estimateR(c)=estimate.r_radps;
  rawSpeed(c)=reference.raw_speed_mps;rawHeading(c)=reference.raw_heading_rad;
  referenceSpeed(c)=reference.speed_mps;referenceHeading(c)=reference.heading_rad;
  fresh(c)=reference.fresh;fallback(c)=reference.fallback;sequence(c)=reference.sequence;
 end
 [nCmd,controller]=controllerBatch(estimateUr,estimatePsi,estimateR, ...
  referenceSpeed,referenceHeading,controller,cfg,N);
 controllerBatches=controllerBatches+1;
 % Exact two-shaft midpoint/end lag; sign masks are per shaft and candidate.
 mid=nCmd+(n-nCmd)*midDecay;
 nextN=nCmd+(n-nCmd)*endDecay;
 coeff=cfg.propeller_positive_coefficient*ones(2,N);
 coeff(mid<0)=cfg.propeller_negative_coefficient;
 force=coeff.*mid.*abs(mid);tauX=sum(force,1);
 tauN=cfg.propeller_lever_m*(force(1,:)-force(2,:));
 du=(tauX+cfg.surge_Xu_N_per_mps*u)/cfg.nominal_mass_matrix(1,1);
 dv=-v/cfg.heading_T_s-u.*r;
 dr=tauN/cfg.nominal_mass_matrix(6,6)-r/cfg.heading_T_s;
 velocityN=u.*cos(psi)-v.*sin(psi)+flow(1);
 velocityE=u.*sin(psi)+v.*cos(psi)+flow(2);
 nextNE=ne+h*[velocityN;velocityE];
 nextPsi=psi+h*r;nextU=u+h*du;nextV=v+h*dv;nextR=r+h*dr;
 nextTime=t+h;
 % Match the scalar per-column finite-state assertion, without reclassifying
 % nonfinite model output as a merely infeasible candidate.
 values=[nextNE;nextPsi;nextU;nextV;nextR;nextN];
 valid=all(isfinite(values),1);
 bad=find(~valid,1,'first');
 if ~isempty(bad)
  error('s04:NominalNonfinite','Nonfinite nominal prediction (ordinal=%d, t=%.17g)',bad,t);
 end
 bodyBatches=bodyBatches+1;
 if k==1,firstCommands=nCmd;end
 referenceRows=[repmat(t,1,N);rawSpeed;rawHeading;referenceHeading; ...
  controller.speed_reference_mps;controller.psi_d_rad;controller.r_d_radps; ...
  nCmd;double(fresh);double(fallback);sequence];
 references(k,:,:)=reshape(referenceRows,1,12,N);
 stateRows=[repmat(nextTime,1,N);nextNE;nextU;nextV;nextPsi;nextR;nextN];
 states(k+1,:,:)=reshape(stateRows,1,9,N);
 ne=nextNE;u=nextU;v=nextV;psi=nextPsi;r=nextR;n=nextN;
end
for c=1:N
 % Rebuild exactly the scalar next-public updates; unconsumed origin fields
 % retain their original values/shapes and field insertion order.
 finalPublic=ctx.public;finalPublic.t_s=nextTime;finalPublic.ne=ne(:,c)';
 finalPublic.psi_rad=psi(c);finalPublic.ur_mps=u(c);finalPublic.vr_mps=v(c);
 finalPublic.r_radps=r(c);finalPublic.n_measured_radps=n(:,c)';
 finalPublic.stop_heading_rad=referenceHeading(c);
 finalPublic.control_estimate=struct('ur_mps',u(c),'psi_rad',psi(c),'r_radps',r(c));
 finalController=struct('speed_reference_mps',controller.speed_reference_mps(c), ...
  'psi_d_rad',controller.psi_d_rad(c),'r_d_radps',controller.r_d_radps(c), ...
  'z_u',controller.z_u(c),'z_psi',controller.z_psi(c));
 rolls{c}=struct('schema_version','s04-reference-event-rollout-v1', ...
  'clock_mode','nominal_renewal','horizon_s',horizon, ...
  'state_trace',states(:,:,c),'state_columns',{{'t_s','north_m','east_m','ur_mps','vr_mps','psi_rad','r_radps','n_left_radps','n_right_radps'}}, ...
  'reference_trace',references(:,:,c),'reference_columns',{{'t_s','raw_ur_ref_mps','raw_psi_ref_rad','shaped_psi_ref_rad', ...
   'limited_ur_ref_mps','filtered_psi_ref_rad','filtered_r_ref_radps','n_cmd_left_radps','n_cmd_right_radps','fresh','fallback','sequence'}}, ...
  'events',{eventLogs{c}},'initial_public',ctx.public,'first_command',firstCommands(:,c)', ...
  'final_public',finalPublic,'final_controller',finalController,'final_rx',rx{c}, ...
  'nominal_first_delivery_s',t0+.05,'reference_core',cores{c}, ...
  'feedback_model','First delayed control estimate; thereafter noise-free instantaneous nominal states', ...
  'scope','Conditional TRACK; held public flow, observed information only; no future guard prediction');
end
stats=struct('N',N,'receiver_calls',receiverCalls,'controller_batches',controllerBatches, ...
 'body_batches',bodyBatches,'math_updates',count*N, ...
 'planned_packets_per_column',syntheticIndex,'model_step_s',h,'horizon_s',horizon);
end

function [nCmd,nextState]=controllerBatch(estimateUr,estimatePsi,estimateR,refSpeed,refHeading,state,cfg,N)
% S08 controller scalar validations are performed per column by the caller.
if isempty(state)
 state=struct('speed_reference_mps',zeros(1,N),'psi_d_rad',estimatePsi, ...
  'r_d_radps',zeros(1,N),'z_u',zeros(1,N),'z_psi',zeros(1,N));
end
h=cfg.control_dt_s;
% S08 nominal_step passes only speed_mps/heading_rad, so fraction is exactly 1.
fraction=1;
validateattributes(fraction,{'double'},{'real','finite','scalar','>',0,'<=',1});
bounded=max(cfg.reference_speed_min_mps,min(cfg.reference_speed_max_mps,refSpeed));
change=max(-cfg.reference_acceleration_down_mps2*h, ...
 min(cfg.reference_acceleration_up_mps2*h,bounded-state.speed_reference_mps));
uRef=state.speed_reference_mps+change;
eu=uRef-estimateUr;
ff=-cfg.surge_Xu_N_per_mps*uRef;
tauXraw=ff+cfg.speed_Kp*eu+cfg.speed_Ki*state.z_u;
tauXclamped=max(cfg.tau_surge_min_N,min(cfg.tau_surge_max_N,tauXraw));
% Frozen LOSobserver/ssa have only scalar constants times candidate row arrays;
% their old-angle differentiator order and mod-based SSA remain unchanged.
[psiD,rD]=LOSobserver(state.psi_d_rad,state.r_d_radps,refHeading,h,cfg.heading_observer_gain);
ePsi=ssa(estimatePsi-psiD);
eRate=estimateR-rD;
tauNraw=(1/cfg.heading_K)*rD-cfg.heading_Kp*(ePsi+cfg.heading_Td_s*eRate+state.z_psi/cfg.heading_Ti_s);
ell=cfg.propeller_lever_m;
forces=[(tauXclamped+tauNraw/ell)/2;(tauXclamped-tauNraw/ell)/2];
coeff=cfg.propeller_positive_coefficient*ones(2,N);
coeff(forces<0)=cfg.propeller_negative_coefficient;
nRaw=sign(forces).*sqrt(abs(forces)./coeff);
commandMin=fraction*cfg.n_min_radps;
commandMax=fraction*cfg.n_max_radps;
nCmd=satlim(nRaw,commandMin,commandMax);
coeff=cfg.propeller_positive_coefficient*ones(size(nCmd));
coeff(nCmd<0)=cfg.propeller_negative_coefficient;
allocatedForces=coeff.*nCmd.*abs(nCmd);
tauAllocated=[sum(allocatedForces,1);ell*(allocatedForces(1,:)-allocatedForces(2,:))];
resX=tauXraw-tauAllocated(1,:);
resN=tauNraw-tauAllocated(2,:);
deltaX=cfg.speed_Ki*eu;
deltaN=-cfg.heading_Kp/cfg.heading_Ti_s*ePsi;
tol=cfg.antiwindup_residual_tolerance;
blockU=abs(resX)>tol & deltaX.*resX>0;
blockPsi=abs(resN)>tol & deltaN.*resN>0;
zU=state.z_u;zPsi=state.z_psi;
activeU=~blockU;activePsi=~blockPsi;
zU(activeU)=max(cfg.speed_integral_min,min(cfg.speed_integral_max,zU(activeU)+h*eu(activeU)));
zPsi(activePsi)=max(cfg.heading_integral_min,min(cfg.heading_integral_max,zPsi(activePsi)+h*ePsi(activePsi)));
nextState=struct('speed_reference_mps',uRef,'psi_d_rad',psiD, ...
 'r_d_radps',rD,'z_u',zU,'z_psi',zPsi);
end

function p=syntheticPacket(core,tg,seq)
% Exact S08 repeated delivery of the SAME absolute reference target.
p=struct('reference_id',sprintf('S04-FORECAST-%06d',seq),'planner_id','L1','sequence',seq, ...
 'generated_from_time',tg,'ready_time',tg+.05,'valid_until',tg+1, ...
 't_ref',tg+(core.t_ref-core.generated_from_time),'psi_ref',core.psi_ref,'ur_ref',core.ur_ref, ...
 'status','ok','failure_reason','');
end
