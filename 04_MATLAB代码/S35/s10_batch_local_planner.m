function decision = s10_batch_local_planner(method,scene,obs,obstacles,previous,ctx,auditCapture,profileCfg)
% S04 isolated reference-aware L1 copy. Candidate order/costs and legacy
% emergency/avoidance heuristics are preserved; VO mathematics is unchanged.
% L1 scores the converted reference through the declared nominal event model.
% Current remains a held PUBLIC estimate, never a future tape.
if nargin<6,ctx=[];end
if nargin<7 || isempty(auditCapture),auditCapture=false;end
if nargin<8,profileCfg=[];end
validateattributes(auditCapture,{'logical'},{'scalar'});
[batchSupported,fallbackReason]=s10PlannerSupport(method,ctx,profileCfg);
if ~batchSupported
    % The runner pins this unmodified function to the frozen S08 source.
    % Exactly one whole search is delegated, never this new wrapper.
    decision=local_planner_p1b(method,scene,obs,obstacles,previous,ctx);
    decision.s10_execution=struct('backend','S08_SCALAR_FALLBACK', ...
        'reason',fallbackReason,'original_l1_calls',double(strcmpi(char(method),'L1')), ...
        'batch_kernel_calls',0,'candidate_count',decision.candidate_count, ...
        'audit_capture',logical(auditCapture),'audit_capture_available',false);
    return;
end
method = upper(char(method));
target = routeLookahead(scene.global_route_ne,obs.ne,scene.lookahead_m);
course = atan2(target(2)-obs.ne(2),target(1)-obs.ne(1));
flow=flowAt(scene);pathHeading=p1b_crab_heading(course,scene.target_speed_mps,flow);
startTimer = tic;
decision = struct('target_speed_mps',scene.target_speed_mps, ...
    'target_heading_rad',pathHeading,'no_solution',false,'emergency',false, ...
    'candidate_count',0,'safe_candidate_count',0,'min_predicted_clearance_m',inf, ...
    'decision_time_s',0,'target_ne',target,'method',method,'reason','route_following','planner_state',struct());

% Public emergency: only a short-horizon physical-contact prediction. It is
% recorded separately and is identical for L0/L1/L2.
emergencyRisk = candidateClearance(obs.ne,obs.psi,max(0,obs.speed),obs.r,3,0.25,obstacles,scene,obs.t_s);
if emergencyRisk < 0
    decision.target_speed_mps = 0;
    decision.target_heading_rad = obs.psi;
    decision.emergency = true;
    decision.reason = 'common_emergency_predicted_contact';
    decision.min_predicted_clearance_m = emergencyRisk;
    decision.decision_time_s = toc(startTimer);
    if strcmp(method,'L0'), return; end
end

switch method
    case 'L0'
        % No local avoidance. This path still uses the common speed/heading controller.
    case 'L1'
        decision = dwaDecision(scene,obs,obstacles,target,pathHeading,previous,decision,startTimer,ctx,auditCapture);
    case 'L2'
        decision = voDecision(scene,obs,obstacles,target,pathHeading,previous,decision,startTimer);
    otherwise
        error('Unknown local method %s.',method);
end
decision.decision_time_s = toc(startTimer);
end

function d = dwaDecision(scene,obs,obstacles,target,pathHeading,previous,d,startTimer,ctx,auditCapture)
assert(isstruct(ctx) && isfield(ctx,'public'),'s04:MissingContext', ...
    'Reference-aware DWA requires the approved public execution context.');
dv = 0.4*scene.decision_dt_s;
dw = 0.35*scene.decision_dt_s;
vLo=max(0,min(scene.maximum_speed_mps,obs.speed-dv));vHi=max(0,min(scene.maximum_speed_mps,obs.speed+dv));
wLo=max(-0.35,obs.r-dw);wHi=min(0.35,obs.r+dw);
speeds=unique([linspace(vLo,vHi,5),0,scene.target_speed_mps]);
yawRates=unique([linspace(wLo,wHi,11),0]);
desiredHeading=pathHeading;
scoreTarget=target;
% Preserved legacy direct-risk/avoidance cue, not the new scoring forecast.
if candidateClearance(obs.ne,obs.psi,max(0.2,obs.speed),0,12,0.4,obstacles,scene,obs.t_s)<scene.safety_margin_m
    desiredHeading=avoidanceHeading(obs.ne,pathHeading,obstacles,obs.t_s);
    scoreTarget=obs.ne+10*[cos(desiredHeading),sin(desiredHeading)];
end
best=inf; found=false; bestClear=-inf;bestV=0;bestW=0;
bestCore=[];bestRoll=[];bestOrdinal=NaN;bestEndNE=[NaN,NaN];bestEndPsi=NaN;
records=struct('ordinal',{},'v_mps',{},'w_radps',{},'core',{}, ...
    'feasible',{},'score',{},'costcomponents',{},'minimum_clearance_m',{}, ...
    'end_ne_m',{},'end_psi_rad',{},'cost_sample_count',{});
% Preserve the exact inherited v-outer/w-inner enumeration before one kernel.
cores=cell(1,numel(speeds)*numel(yawRates));coreOrdinal=0;
for coreI=1:numel(speeds)
    for coreJ=1:numel(yawRates)
        coreOrdinal=coreOrdinal+1;
        cores{coreOrdinal}=s04_candidate_to_reference(speeds(coreI),yawRates(coreJ),obs.psi,obs.t_s);
    end
end
[allRolls,batchStats]=s10_batch_reference_rollout(cores,ctx);
assert(iscell(allRolls) && isequal(size(allRolls),[1,numel(cores)]), ...
    's10:BatchRollShape','Expected one canonical roll per original candidate ordinal.');
sampleIndices=1+(1:30)*8;
for i=1:numel(speeds)
    for j=1:numel(yawRates)
        v=speeds(i);w=yawRates(j);d.candidate_count=d.candidate_count+1;
        % Convert before forecasting. The saved winning core is packaged as-is.
        core=cores{d.candidate_count};
        roll=allRolls{d.candidate_count};
        states=roll.state_trace;
        assert(isequal(size(states),[241,9]) && all(isfinite(states(:))), ...
            's04:ForecastStateShape','Expected finite 241-by-9 nominal state trace.');
        assert(isequal(size(roll.reference_trace),[240,12]), ...
            's04:ForecastReferenceShape','Expected 240-by-12 reference trace.');
        assert(max(abs(states(sampleIndices,1)-(obs.t_s+(1:30)'*0.4)))<1e-10, ...
            's04:ForecastSamplingClock','Scoring samples must match original absolute times.');
        clearance=inf;pathPenalty=0;
        for k=1:30
            p=states(sampleIndices(k),2:3);
            % Exact inherited point-clearance semantics and observed-object clock.
            clearance=min(clearance,clearanceAt(p,obstacles,scene,obs.t_s+k*0.4));
            pathPenalty=pathPenalty+polylineDistance(p,scene.global_route_ne)*0.4/12;
        end
        endNE=states(end,2:3);endPsi=states(end,6);
        feasible=~(clearance < scene.safety_margin_m);
        components=struct('goal',NaN,'path',pathPenalty,'heading',NaN, ...
            'speed_v',v,'change',NaN,'clearance',NaN);
        score=inf;
        if feasible
            d.safe_candidate_count=d.safe_candidate_count+1;
            goalCost=hypot(endNE(1)-scoreTarget(1),endNE(2)-scoreTarget(2));
            headingCost=abs(wrapPi(endPsi-desiredHeading));
            changeCost=abs(v-previous.target_speed_mps)+0.5*abs(w-obs.r);
            clearanceCost=1/max(clearance-scene.safety_margin_m+0.25,0.25);
            score=1.4*goalCost+0.4*pathPenalty+1.5*headingCost-4.0*v+0.3*changeCost+0.5*clearanceCost;
            components=struct('goal',goalCost,'path',pathPenalty,'heading',headingCost, ...
                'speed_v',v,'change',changeCost,'clearance',clearanceCost);
        end
        rec=struct('ordinal',d.candidate_count,'v_mps',v,'w_radps',w,'core',core, ...
            'feasible',logical(feasible),'score',score,'costcomponents',components, ...
            'minimum_clearance_m',clearance,'end_ne_m',endNE,'end_psi_rad',endPsi, ...
            'cost_sample_count',30);
        records(end+1)=rec; %#ok<AGROW>
        % Inherited first strict minimum, with original v-outer/w-inner order.
        if feasible && score < best
            best=score;bestV=v;bestW=w;bestClear=clearance;found=true;
            bestCore=core;bestRoll=roll;bestOrdinal=d.candidate_count;
            bestEndNE=endNE;bestEndPsi=endPsi;
        end
    end
end
if found
    d.reference_core=bestCore;
    d.target_speed_mps=bestCore.ur_ref_mps;
    d.target_heading_rad=bestCore.psi_ref_rad;
    d.s04_forecast=bestRoll;
    d.min_predicted_clearance_m=bestClear;
    d.reason='dwa_reference_safe_minimum_cost';
    coreSource='scored_winner';
else
    % This is the inherited failed/fallback action, not a scoring winner.
    d.reference_core=s04_candidate_to_reference(0,0,obs.psi,obs.t_s);
    d.target_speed_mps=d.reference_core.ur_ref_mps;
    d.target_heading_rad=d.reference_core.psi_ref_rad;
    d.s04_forecast=[];
    d.no_solution=true;d.emergency=true;
    d.min_predicted_clearance_m=-inf;d.reason='dwa_no_safe_candidate';
    coreSource='fallback_not_scored';
end
d.search_speed_min_mps=min(speeds);d.search_speed_max_mps=max(speeds);
d.s04_candidate_records=records;
if auditCapture,d.s10_audit_all_rolls=allRolls;end
d.s10_execution=struct('backend','BATCH','reason','', ...
    'original_l1_calls',0,'batch_kernel_calls',1,'candidate_count',d.candidate_count, ...
    'audit_capture',logical(auditCapture),'audit_capture_available',logical(auditCapture), ...
    'batch_stats',batchStats);
d.s04_dwa_observation=struct('schema_version','s04-reference-dwa', ...
    'winner_found',logical(found),'winner_v_mps',NaN,'winner_w_radps',NaN, ...
    'winner_ordinal',bestOrdinal,'winner_score',best, ...
    'origin_ne_m',obs.ne,'origin_psi_rad',obs.psi,'origin_t_s',obs.t_s, ...
    'origin_search_ur_mps',obs.speed,'origin_public_ur_mps',ctx.public.ur_mps, ...
    'origin_r_radps',obs.r,'winner_end_ne_m',bestEndNE,'winner_end_psi_rad',bestEndPsi, ...
    'minimum_scored_clearance_m',d.min_predicted_clearance_m, ...
    'prediction_horizon_s',12,'model_step_s',0.05,'cost_sample_step_s',0.4, ...
    'cost_sample_count',30,'candidate_count',d.candidate_count, ...
    'reference_core_source',coreSource, ...
    'forecast_policy','CONDITIONAL_REPEATED_ABSOLUTE_REFERENCE', ...
    'legacy_emergency_and_avoidance_preserved',true, ...
    'cost_weights',struct('goal',1.4,'path',0.4,'heading',1.5, ...
        'speed_v',-4.0,'change',0.3,'clearance',0.5));
if found
    d.s04_dwa_observation.winner_v_mps=bestV;
    d.s04_dwa_observation.winner_w_radps=bestW;
end
d.decision_time_s=toc(startTimer);
end

function d = voDecision(scene,obs,obstacles,target,pathHeading,previous,d,startTimer)
% Sample admissible velocities and reject those whose finite-horizon swept
% footprint enters an obstacle velocity cone. Heading/speed change are bounded.
dv=0.35*scene.decision_dt_s;
maxHeadingStep=deg2rad(18);
vLo=max(0,min(scene.maximum_speed_mps,obs.speed-dv));vHi=max(0,min(scene.maximum_speed_mps,obs.speed+dv));
speeds=unique([linspace(vLo,vHi,6),0,scene.target_speed_mps]);
center=wrapPi(pathHeading);
scoreTarget=target;
if candidateClearance(obs.ne,obs.psi,max(0.2,obs.speed),0,15,0.4,obstacles,scene,obs.t_s)<scene.safety_margin_m
    center=avoidanceHeading(obs.ne,pathHeading,obstacles,obs.t_s);
    scoreTarget=obs.ne+10*[cos(center),sin(center)];
end
headings=unique([linspace(obs.psi-maxHeadingStep,obs.psi+maxHeadingStep,17), ...
    max(obs.psi-maxHeadingStep,min(obs.psi+maxHeadingStep,center)),obs.psi]);
best=inf;found=false;bestClear=-inf;bestV=0;bestPsi=obs.psi;
for i=1:numel(speeds)
    for j=1:numel(headings)
        v=speeds(i);psi=wrapPi(headings(j));d.candidate_count=d.candidate_count+1;
        [clearance,endNE,~,pathPenalty]=velocityCandidate(obs.ne,obs.psi,obs.speed,v,psi,15,0.4,obstacles,scene,obs.t_s);
        if clearance < scene.safety_margin_m, continue; end
        d.safe_candidate_count=d.safe_candidate_count+1;
        goalCost=hypot(endNE(1)-scoreTarget(1),endNE(2)-scoreTarget(2));
        headingCost=abs(wrapPi(psi-center));
        changeCost=abs(v-previous.target_speed_mps)+abs(wrapPi(psi-previous.target_heading_rad));
        clearanceCost=1/max(clearance-scene.safety_margin_m+0.25,0.25);
        score=1.3*goalCost+0.4*pathPenalty+1.2*headingCost-4.0*v+0.5*changeCost+0.5*clearanceCost;
        if score<best
            best=score;bestV=v;bestPsi=psi;bestClear=clearance;found=true;
        end
    end
end
if found
    d.target_speed_mps=bestV;d.target_heading_rad=bestPsi;
    d.min_predicted_clearance_m=bestClear;d.reason='vo_admissible_minimum_cost';
else
    d.target_speed_mps=0;d.target_heading_rad=obs.psi;d.no_solution=true;d.emergency=true;
    d.min_predicted_clearance_m=-inf;d.reason='vo_no_admissible_velocity';
end
d.search_speed_min_mps=min(speeds);d.search_speed_max_mps=max(speeds);
d.decision_time_s=toc(startTimer);
end

function [minimum,endNE,endPsi,pathPenalty] = candidateClearance(ne,psi,v,w,horizon,dt,obstacles,scene,t0)
p=ne;ang=psi;minimum=inf;pathPenalty=0;n=max(1,ceil(horizon/dt));
for k=1:n
    ang=wrapPi(ang+w*dt);p=p+dt*(v*[cos(ang),sin(ang)]+flowAt(scene));
    minimum=min(minimum,clearanceAt(p,obstacles,scene,t0+k*dt));
    pathPenalty=pathPenalty+polylineDistance(p,scene.global_route_ne)*dt/horizon;
end
endNE=p;endPsi=ang;
end

function [minimum,endNE,endPsi,pathPenalty] = velocityCandidate(ne,psi0,v0,vTarget,psiTarget,horizon,dt,obstacles,scene,t0)
p=ne;ang=psi0;v=v0;minimum=inf;pathPenalty=0;n=max(1,ceil(horizon/dt));
for k=1:n
    ang=moveAngle(ang,psiTarget,deg2rad(18)*dt);
    v=v+max(-0.35*dt,min(0.35*dt,vTarget-v));
    p=p+dt*(v*[cos(ang),sin(ang)]+flowAt(scene));
    minimum=min(minimum,clearanceAt(p,obstacles,scene,t0+k*dt));
    pathPenalty=pathPenalty+polylineDistance(p,scene.global_route_ne)*dt/horizon;
end
endNE=p;endPsi=ang;
end

function c=clearanceAt(ne,obstacles,scene,t)
c=inf;
for i=1:numel(obstacles)
    o=obstacles(i);
    if strcmp(o.type,'rect')
        r=o.bounds_ne;dn=max([r(1)-ne(1),0,ne(1)-r(2)]);de=max([r(3)-ne(2),0,ne(2)-r(4)]);
        c=min(c,hypot(dn,de)-scene.footprint_radius_m);
    else
        center=o.initial_ne+(t-o.observed_at_s)*o.velocity_ne_mps;
        c=min(c,hypot(ne(1)-center(1),ne(2)-center(2))-scene.footprint_radius_m-o.radius_m);
    end
end
end

function [target,s]=routeLookahead(route,ne,lookahead)
[~,s0]=polylineDistance(ne,route);seg=hypot(diff(route(:,1)),diff(route(:,2)));cum=[0;cumsum(seg)];s=min(cum(end),s0+lookahead);
idx=find(cum<=s,1,'last');if idx>=size(route,1),target=route(end,:);return;end
u=(s-cum(idx))/seg(idx);target=route(idx,:)+u*(route(idx+1,:)-route(idx,:));
end

function [d,bestS]=polylineDistance(ne,route)
seg=hypot(diff(route(:,1)),diff(route(:,2)));cum=[0;cumsum(seg)];d=inf;bestS=0;
for i=1:size(route,1)-1
    a=route(i,:);v=route(i+1,:)-a;u=max(0,min(1,dot(ne-a,v)/dot(v,v)));q=a+u*v;z=hypot(ne(1)-q(1),ne(2)-q(2));
    if z<d,d=z;bestS=cum(i)+u*seg(i);end
end
end

function a=moveAngle(a,target,maxStep)
d=wrapPi(target-a);a=wrapPi(a+max(-maxStep,min(maxStep,d)));
end

function heading=avoidanceHeading(ne,pathHeading,obstacles,t)
% Deterministic pass-side cue used only when the direct finite-horizon velocity
% is unsafe. The collision test still decides feasibility.
best=inf;side=1;
for i=1:numel(obstacles)
    o=obstacles(i);
    if strcmp(o.type,'rect')
        center=[mean(o.bounds_ne(1:2)),mean(o.bounds_ne(3:4))];
    else
        center=o.initial_ne+(t-o.observed_at_s)*o.velocity_ne_mps;
    end
    delta=center-ne;dist=hypot(delta(1),delta(2));
    if dist<best
        best=dist;cross=cos(pathHeading)*delta(2)-sin(pathHeading)*delta(1);
        if cross>0,side=-1;else,side=1;end
    end
end
heading=wrapPi(pathHeading+side*pi/2);
end

function a=wrapPi(a)
a=atan2(sin(a),cos(a));
end

function flow=flowAt(scene)
flow=[0,0];if isfield(scene,'current_estimate_ne_mps'),flow=reshape(scene.current_estimate_ne_mps,1,2);end
end


function [supported,reason]=s10PlannerSupport(method,ctx,profileCfg)
% Pure preflight only: no I/O, no scalar/kernel execution, no state mutation.
% A non-TRACK origin is already outside frozen S08's forecast domain.
supported=false;reason='';methodTag=upper(char(method));
if strcmp(methodTag,'L1') && isstruct(ctx) && isscalar(ctx) && isfield(ctx,'mode')
    assert(any(strcmp(ctx.mode,{'','TRACK'})), ...
        's04:UnsupportedOriginMode','Only TRACK origin is qualified here');
end
if ~strcmp(methodTag,'L1')
    reason='METHOD_NOT_IN_BATCH_PROFILE';return;
end
if isempty(profileCfg) || ~isstruct(ctx) || ~isscalar(ctx) || ...
        ~isfield(ctx,'cfg') || ~isequaln(ctx.cfg,profileCfg)
    reason='CONFIG_NOT_IN_BATCH_PROFILE';return;
end
if ~isfield(ctx,'controller')
    reason='CONTROLLER_SCHEMA_NOT_IN_BATCH_PROFILE';return;
end
if ~isempty(ctx.controller)
    expected={'speed_reference_mps';'psi_d_rad';'r_d_radps';'z_u';'z_psi'};
    if ~isstruct(ctx.controller) || ~isscalar(ctx.controller) || ...
            ~isequal(sort(fieldnames(ctx.controller)),sort(expected))
        reason='CONTROLLER_SCHEMA_NOT_IN_BATCH_PROFILE';return;
    end
    for k=1:numel(expected)
        if ~s10FiniteDouble(ctx.controller.(expected{k}),1)
            reason='CONTROLLER_STATE_NOT_IN_BATCH_PROFILE';return;
        end
    end
end
if ~isfield(ctx,'public') || ~isstruct(ctx.public) || ~isscalar(ctx.public)
    reason='PUBLIC_STATE_NOT_IN_BATCH_PROFILE';return;
end
scalarFields={'t_s','ur_mps','vr_mps','psi_rad','r_radps'};
for k=1:numel(scalarFields)
    key=scalarFields{k};
    if ~isfield(ctx.public,key) || ~s10FiniteDouble(ctx.public.(key),1)
        reason='PUBLIC_STATE_NOT_IN_BATCH_PROFILE';return;
    end
end
vectorFields={'ne','n_measured_radps','current_estimate_ne_mps'};
for k=1:numel(vectorFields)
    key=vectorFields{k};
    if ~isfield(ctx.public,key) || ~s10FiniteDouble(ctx.public.(key),2)
        reason='PUBLIC_STATE_NOT_IN_BATCH_PROFILE';return;
    end
end
if ~isfield(ctx.public,'control_estimate') || ...
        ~isstruct(ctx.public.control_estimate) || ~isscalar(ctx.public.control_estimate)
    reason='CONTROL_ESTIMATE_NOT_IN_BATCH_PROFILE';return;
end
for key={'ur_mps','psi_rad','r_radps'}
    if ~isfield(ctx.public.control_estimate,key{1}) || ...
            ~s10FiniteDouble(ctx.public.control_estimate.(key{1}),1)
        reason='CONTROL_ESTIMATE_NOT_IN_BATCH_PROFILE';return;
    end
end
supported=true;
end

function ok=s10FiniteDouble(value,count)
ok=isa(value,'double') && isreal(value) && isvector(value) && ...
    numel(value)==count && all(isfinite(value(:)));
end
