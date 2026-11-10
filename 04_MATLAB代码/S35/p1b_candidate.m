function [packet,job]=p1b_candidate(method,public,objects,scene,previous,epoch,t,s,ctx)
% S04: optional common execution context; original scene whitelist retained.
outerTimer=tic;
if nargin<9,ctx=[];end
names={'id','bounds_ne','start_ne','goal_ne','global_route_ne','footprint_radius_m','safety_margin_m', ...
    'sensor_range_m','lookahead_m','target_speed_mps','maximum_speed_mps','decision_dt_s','control_dt_s'};
candidateScene=struct();for k=1:numel(names),candidateScene.(names{k})=scene.(names{k});end
candidateScene.current_estimate_ne_mps=public.current_estimate_ne_mps;
% Snapshot/assert outside the original decision-call timing boundary.
contextBefore=ctx;rngBefore=rng();
timer=tic;
if strcmp(method,'ASTAR')
    assert(~isempty(which('p1b_astar_candidate')),'p1b:MissingCandidate','ASTAR helper not installed yet.');
    candidateSettings=struct('control_dt_s',s.control_dt_s,'speed_max_mps',s.speed_max_mps, ...
        'protocol',struct('candidate_contract',s.protocol.candidate_contract));
    candidateSettings.execution_context=ctx; % Same permission; native A* math unchanged.
    decision=p1b_astar_candidate(candidateScene,public,objects,previous,candidateSettings);
else
    obs=struct('t_s',t,'ne',public.ne,'psi',public.psi_rad,'speed',max(0,public.ur_mps),'r',public.r_radps);
    if strcmp(method,'L1')
        decision=s10_batch_local_planner(method,candidateScene,obs,objects,previous,ctx,false,ctx.cfg);
    else
        decision=local_planner_p1b(method,candidateScene,obs,objects,previous,ctx);
    end
end
cpu=toc(timer);
rngAfter=rng();contextUnchanged=isequaln(contextBefore,ctx);rngUnchanged=isequaln(rngBefore,rngAfter);
assert(contextUnchanged,'s04:ContextMutation','Candidate changed its caller execution context.');
assert(rngUnchanged,'s04:RNGMutation','Candidate changed the caller RNG state.');
assert(isfinite(decision.target_speed_mps) && decision.target_speed_mps>=0 && decision.target_speed_mps<=1.2+1e-12,'p1b:CandidateCap','Candidate UR outside common bound');
if ~isfield(decision,'planner_state'),decision.planner_state=struct();end
scoringCore=[];
if strcmp(method,'L1') && ~isempty(ctx)
    assert(isfield(decision,'reference_core'),'s04:MissingReferenceCore','Adapted L1 must retain its scoring reference core, including failure outcomes.');
    scoringCore=decision.reference_core;
    requiredCore={'t_ref','psi_ref','ur_ref','generated_from_time','valid_until'};
    assert(isstruct(scoringCore) && isscalar(scoringCore) && all(isfield(scoringCore,requiredCore)), ...
        's04:InvalidReferenceCore','Scoring reference core is incomplete.');
    assert(numel(scoringCore.t_ref)==31 && numel(scoringCore.psi_ref)==31 && numel(scoringCore.ur_ref)==31, ...
        's04:ReferenceCoreNodes','Scoring reference core must retain all 31 native nodes.');
    % Independently construct production packet from the SAVED scoring core.
    % Do not recalculate heading, speed, node times or validity from the decision.
    packet=struct('reference_id',sprintf('P1B-%s-%06d',method,epoch),'planner_id',method,'sequence',epoch, ...
        'generated_from_time',scoringCore.generated_from_time,'ready_time',t,'valid_until',scoringCore.valid_until, ...
        't_ref',scoringCore.t_ref,'psi_ref',scoringCore.psi_ref,'ur_ref',scoringCore.ur_ref,'status','ok','failure_reason','');
else
    % Unadapted/other-method packet mathematics is the original production path.
    psi=atan2(sin(decision.target_heading_rad),cos(decision.target_heading_rad));
    packet=struct('reference_id',sprintf('P1B-%s-%06d',method,epoch),'planner_id',method,'sequence',epoch, ...
        'generated_from_time',t,'ready_time',t,'valid_until',t+s.reference_ttl_s, ...
        't_ref',t+(0:30)*.5,'psi_ref',repmat(psi,1,31),'ur_ref',repmat(decision.target_speed_mps,1,31),'status','ok','failure_reason','');
end
if decision.no_solution,packet.status='failed';packet.failure_reason=decision.reason;end
productionArrays=struct('t_ref',packet.t_ref,'psi_ref',packet.psi_ref,'ur_ref',packet.ur_ref, ...
    'generated_from_time',packet.generated_from_time,'valid_until',packet.valid_until);
identity=struct('used_saved_scoring_core',~isempty(scoringCore),'all_equal',false);
if ~isempty(scoringCore)
    identity.t_ref_equal=isequaln(scoringCore.t_ref,productionArrays.t_ref);
    identity.psi_ref_equal=isequaln(scoringCore.psi_ref,productionArrays.psi_ref);
    identity.ur_ref_equal=isequaln(scoringCore.ur_ref,productionArrays.ur_ref);
    identity.generated_from_time_equal=isequaln(scoringCore.generated_from_time,productionArrays.generated_from_time);
    identity.valid_until_equal=isequaln(scoringCore.valid_until,productionArrays.valid_until);
    identity.all_equal=identity.t_ref_equal && identity.psi_ref_equal && identity.ur_ref_equal && ...
        identity.generated_from_time_equal && identity.valid_until_equal;
    assert(identity.all_equal,'s04:ReferenceCoreMismatch','Production reference differs from saved scoring core.');
end
job=struct('method',method,'epoch',epoch,'generated_from_time',t,'actual_cpu_s',cpu, ...
    'public_launch',public,'observed_objects',objects,'candidate_scene',candidateScene,'previous',previous, ...
    'decision',decision,'speed_semantics','water-relative surge; current-aware ground prediction');
job.context_isolation=struct('context_unchanged',contextUnchanged,'rng_unchanged',rngUnchanged, ...
    'assertion_scope','Exact isequaln at the decision-call boundary');
job.execution_context=ctx;
job.scoring_reference_core=scoringCore;
job.production_reference_arrays=productionArrays;
job.reference_core_identity=identity;
% Actual ready/TTL/delivery scheduling remains exclusively in the native engine.
job.outer_total_s=toc(outerTimer);
end
