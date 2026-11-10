function summary=run_s27_pair(base)
% S27 pair with one shared 9.6 m lookahead under S24 controlled delivery.
% Each case starts from its original frozen typed S24 initial state.
entryClock=tic;approval='5190f07d-8433-509e-bcc8-2709e04e3642';
repo='/Users/sam/.codex/.chatgpt-projects/g-p-6a97c4bed9b88191a88664b2d07b23d1/asv-experiment-viewer';
runDir=fullfile(base,'run_001');
assert(isfolder(base) && ~isfolder(runDir),'s27:OutputConflict');
% Validate launch and all pinned source bytes before setup.
launch=launchGate(base);freezePath=getenv('ASV27_FREEZE');
freezeSha=fileHash(freezePath);assert(strcmp(freezeSha,getenv('ASV27_FREEZE_SHA')),'s27:FreezeSelfHash');
freeze=jsondecode(fileread(freezePath));assert(strcmp(freeze.schema,'s27-freeze-v1') && strcmp(freeze.approval_message,approval),'s27:FreezeIdentity');
before=verifyFreeze(freeze.files);
approvalPath=fullfile(base,'audit','approval.md');requireFrozen(approvalPath,freeze.files);
assert(contains(fileread(approvalPath),approval),'s27:ApprovalFile');
mkdir(runDir);writeJson(fullfile(runDir,'dependencies_before.json'),before);
writeJson(fullfile(runDir,'launch_environment.json'),launch);
counts=zeroCounts();durableCounts=zeroCounts();upper=authorizedUpper();
summary=struct('schema','s27-common-lookahead-qualification-v1','approval_message',approval,'status','STARTED', ...
 'stage','input_gate','started_utc',utcNow(),'case_order',{{'ASTAR','THETA_STAR'}}, ...
 'extra_warmup_calls',0,'scalar_oracle_calls',0,'posthoc_model_calls',0,'collector_calls',0, ...
 'independent_controller_reproduction_calls',0,'process_hard_cap_s',1800,'entry_soft_cap_s',1500, ...
 'freeze_path',freezePath,'freeze_sha256',freezeSha,'source_before_verified',true, ...
 'timing_contract_label','controlled_constant_0.05','engine_timing_mode','replay','controlled_latency_s',.05, ...
 'actual_lookahead_m',9.6,'qualification_status','NOT_COMPLETED');
cfg=[];settings=[];profileCfg=[];c=[];reference=[];goldenSettings=[];inputProvenance=[];
startupDecision=[];startupResult=[];startupBefore=[];startupAfter=[];startupRngBefore=[];startupRngAfter=[];
common=[];commonCase=[];externalRng=[];postdrawRng=[];C=[];initial=[];currentBeforeStep=[];
rows={};jobTimingRows={};caseIndex=0;chunkIndex=0;chunkPaths={};caseResults=cell(1,2);currentPreflight=[];preflightCtx=[];
currentFolder='';currentStats=[];actualInputs=[];globalData=[];spec=[];proposal=[];resolution=[];after=[];
wholeBatchStop=false;unreturnedAdvance=false;writeWall=0;caseStarted=false;setupCompleted=false;labels={'ASTAR','THETA_STAR'};
try
 specPath=fullfile(base,'input_spec.json');requireFrozen(specPath,freeze.files);
 spec=jsondecode(fileread(specPath));proposal=assertSpec(base,spec,approval,freeze.files);
 [saved,globalData,reuseProvenance]=s27_load_inputs(base,freeze.files,spec,proposal);
 S24_initials=saved.S24_initials;S27_clones=saved.S27_clones;cloneProofs=saved.clone_evidence;
 save(fullfile(runDir,'typed_clone_inputs.mat'),'S24_initials','S27_clones','cloneProofs','-v7');
 writeJson(fullfile(runDir,'clone_normalization.json'),cloneProofs);
 common=saved.common;commonCase=saved.commonCase;externalRng=saved.externalRng;postdrawRng=saved.postdrawRng;
 assert(strcmp(reuseProvenance.common.path,spec.common_source_path) && ...
  strcmp(reuseProvenance.global.path,spec.global_source_path) && ...
  strcmp(reuseProvenance.common.sha256,spec.common_source_sha256) && ...
  strcmp(reuseProvenance.global.sha256,spec.global_source_sha256),'s27:WrapperSources');
 assertSceneMap(globalData.scene_map);
 actualInputs=struct('global_phase_path',reuseProvenance.global.path,'global_phase_sha256',reuseProvenance.global.sha256, ...
  'global_phase_json_text',reuseProvenance.global_text,'global_results',globalData,'input_spec',spec,'approved_proposal',proposal, ...
  'common_path',reuseProvenance.common.path,'common_sha256',reuseProvenance.common.sha256, ...
  'actual_lookahead_m',spec.registered_lookahead_m,'S24_initial_sources',{reuseProvenance.S24_initial_sources}, ...
  'timing_contract_label','controlled_constant_0.05','engine_timing_mode','replay','controlled_latency_s',.05);
 writeJson(fullfile(runDir,'actual_global_inputs.json'),actualInputs);
 [c,profileCfg,reference,goldenSettings,inputProvenance]=s21_load_inputs(base);
 for j=1:numel(inputProvenance.sources)
  requireFrozen(inputProvenance.sources(j).path,freeze.files);
 end
 inputProvenance.reused_common=rmfield(reuseProvenance,'global_text');
 summary.stage='one_q1_setup';checkSoft(entryClock);counts.setup_begun=1;journal('before_setup');
 addpath(fullfile(repo,'experiments','q1_route_geometry_pilot_v1'),'-begin');
 clear q1_setup
 assert(strcmp(which('q1_setup'),fullfile(repo,'experiments','q1_route_geometry_pilot_v1','q1_setup.m')),'s27:SetupResolutionBeforeCall');
 requireFrozen(which('q1_setup'),freeze.files);
 [cfg,environment,settings]=q1_setup();counts.setup_returned=1;counts.setup_BOTH_rhs_on_return=1;counts.setup_archived_otter_on_return=1;setupCompleted=true;
 addpath(fullfile(fileparts(base),'S08','code'),'-begin');addpath(fullfile(base,'code'),'-begin');
 clear p1b_engine p1b_candidate s10_batch_local_planner s10_batch_reference_rollout s13_forecast_receiver s10_compare_science s21_load_inputs s21_preflight
 assert(isequaln(cfg,profileCfg) && isequaln(settings,goldenSettings) && ...
  isequaln(cfg,saved.cfg) && isequaln(settings,saved.settings),'s27:SharedConfiguration');
 assertRecoveryZero(common,settings);
 assert(settings.control_dt_s==.05 && settings.compute_budget_s==.5 && settings.recovery.minimum_brake_s==1 && ...
  settings.nominal_horizon_s==4 && settings.stop_horizon_s==6,'s27:RecoveryZeroPremise');
 p=profile('status');assert(strcmp(p.ProfilerStatus,'off'),'s27:ProfilerActive');
 resolution=pinResolution(base,repo,freeze.files);
 writeJson(fullfile(runDir,'runtime_resolution.json'),resolution);
 writeJson(fullfile(runDir,'input_provenance.json'),inputProvenance);
 save(fullfile(runDir,'typed_inputs_and_runtime_configuration.mat'),'c','profileCfg','reference','goldenSettings', ...
  'cfg','settings','environment','actualInputs','inputProvenance','spec','proposal','-v7');
 durableCounts=counts;journal('runtime_configuration_saved');

 % Sole registered first target call in this process. Cold time alone may fail.
 summary.stage='registered_frozen_A_D1_startup';checkSoft(entryClock);
 [gate,~]=s21_preflight(c.ctx,profileCfg,profileCfg);assert(gate.supported,'s27:StartupPreflight');
 ctx=c.ctx;scene=c.scene;obs=c.obs;objects=c.objects;previous=c.previous;
 rng(c.rng);counts.rng_state_restores=counts.rng_state_restores+1;startupRngBefore=rng();startupBefore=inputSnapshot(ctx,scene,obs,objects,previous);
 counts.startup_l1_begun=1;journal('before_startup');timer=tic;
 startupDecision=s10_batch_local_planner('L1',scene,obs,objects,previous,ctx,false,profileCfg);
 startupWall=toc(timer);counts.startup_l1_returned=1;
 counts=addDecisionWork(counts,startupDecision);journal('after_startup_return');
 startupAfter=inputSnapshot(ctx,scene,obs,objects,previous);startupRngAfter=rng();
 comparison=s10_compare_science(reference,startupDecision,{},{});
 isolation=struct('all_inputs_exact_equal',isequaln(startupBefore,startupAfter), ...
  'rng_exact_equal',isequaln(startupRngBefore,startupRngAfter),'frozen_rng_exact',isequaln(c.rng,startupRngBefore));
 startupResult=struct('wall_s',startupWall,'budget_s',.5,'time_budget_pass',startupWall<=.5, ...
  'time_only_overbudget_may_continue',true,'comparison',comparison,'isolation',isolation, ...
  'candidate_count',startupDecision.candidate_count,'condition','Registered first L1; no selector warmup; subsequent cases remain ordered conditions.');
 save(fullfile(runDir,'startup.mat'),'startupDecision','startupResult','startupBefore','startupAfter','startupRngBefore','startupRngAfter','-v7');
 writeJson(fullfile(runDir,'startup_result.json'),startupResult);durableCounts=counts;journal('startup_saved');
 verifyProduction(startupDecision,true);
 assert(comparison.passed && isolation.all_inputs_exact_equal && isolation.rng_exact_equal && isolation.frozen_rng_exact,'s27:StartupScienceOrIsolation');
 summary.startup_result=startupResult;

 % Archive the typed original before creating either independent clone.
 summary.stage='saved_common_reuse';checkSoft(entryClock);
 noiseEvidence=struct('shape',[1804,4],'normal_values_reused',7216,'seed_provenance',1103, ...
  'draw_calls',0,'common_initializations',0,'stored_postdraw_equals_C_rng_state',isequaln(postdrawRng,common.rng_state), ...
  'source',reuseProvenance.common,'external_RNG_restore_role','Restore stored E before each clone; no scientific draw');
 save(fullfile(runDir,'reused_common.mat'),'common','commonCase','externalRng','postdrawRng','noiseEvidence','cfg','settings','-v7');
 writeJson(fullfile(runDir,'common_noise_evidence.json'),noiseEvidence);journal('saved_common_durable');

 for ci=1:2
  caseIndex=ci;caseStarted=false;rows={};jobTimingRows={};currentFolder='';chunkPaths={};chunkIndex=0;currentStats=[];
  m=globalData.methods(ci);caseId=m.method;summary.stage=['case_' caseId];
  checkSoft(entryClock);verifyFreeze(freeze.files);assert(strcmp(fileHash(freezePath),freezeSha),'s27:FreezeChanged');
  C=S27_clones{ci};initial=C;cloneEvidence=cloneProofs{ci};
  assert(strcmp(C.case_definition.id,['S27_' caseId]) && C.case_definition.scene.lookahead_m==9.6 && ...
   cloneEvidence.whole_normalized_isequaln && cloneEvidence.recursive_fields_classes_shapes_and_values_equal_after_normalization, ...
   's27:CloneLaunchBinding');
  assertRecoveryZero(C,settings);rng(externalRng);counts.rng_state_restores=counts.rng_state_restores+1;
  assert(isequaln(rng(),externalRng),'s27:RestoreStoredExternalRng');
  currentFolder=fullfile(runDir,caseId);mkdir(currentFolder);caseStarted=true;currentStats=zeroStats(initial);
  counts.case_attempts_begun=counts.case_attempts_begun+1;
  assert(counts.case_attempts_begun<=2,'s27:CaseAttemptBound');
  save(fullfile(currentFolder,'initial_state.mat'),'initial','cfg','settings','cloneEvidence','externalRng','postdrawRng','-v7');
  writeJson(fullfile(currentFolder,'clone_contract.json'),cloneEvidence);
  reason='RUNNING';stopDetail=struct();
  while C.next_tick<1800
   if toc(entryClock)>=1500,reason='ENGINEERING_SOFT_WALL';wholeBatchStop=true;break;end
   assertPhysical(C,[]);assert(~C.finished && ~C.collision_pending && ~C.goal_pending,'s27:UnexpectedTerminalEntry');
   currentBeforeStep=C;currentPreflight=[];preflightCtx=[];dispatchExpected=mod(C.next_tick,10)==0;
   if dispatchExpected
    [currentPreflight,preflightCtx]=s21_preflight(C,cfg,profileCfg);
    if ~currentPreflight.supported
     reason='PRETARGET_SUPPORT_STOP';stopDetail=currentPreflight;
     save(fullfile(currentFolder,'pretarget_support_stop.mat'),'C','currentPreflight','preflightCtx','counts','-v7');break;
    end
    counts.native_dispatch_begun=counts.native_dispatch_begun+1;
   end
   assert(counts.advance_begun<3600 && counts.native_dispatch_begun<=360,'s27:AuthorizedCallBound');
   counts.advance_begun=counts.advance_begun+1;unreturnedAdvance=true;journal('before_advance');
   [C,row]=p1b_engine('step',C,cfg,settings);
   unreturnedAdvance=false;counts.advance_returned=counts.advance_returned+1;
   rows{end+1}=row; %#ok<AGROW> every returned row retained before validation
   counts=addReturnedWork(counts,row);journal('after_advance_return');
   assert(~isempty(fieldnames(row.action)) && row.action.interval_completed==1,'s27:EmptyOrIncompleteReturnedInterval');
   assert(row.tick==currentBeforeStep.next_tick && C.next_tick==row.tick+1 && abs(row.t_s-row.tick*.05)<=1e-12 && ...
    abs(row.state_after(2)-C.next_tick*.05)<=1e-12,'s27:NativeClock');
   assertPhysical(C,row);assert(isequaln(C.external_noise_tape,common.external_noise_tape) && ...
    isequaln(C.rng_state,common.rng_state) && isequaln(rng(),externalRng),'s27:SharedRngOrNoiseMutation');
   assert(row.observation.noise_tape_index==C.next_tick && C.external_noise_index==C.next_tick+1 && ...
    C.sensor_sample_count==C.next_tick,'s27:NoiseIndex');
   assert(numel(row.job_bundles)==double(dispatchExpected),'s27:UnexpectedDispatch');
   assert(row.prediction.map.lookahead_m==9.6,'s27:ResolvedMapLookahead');
   timing=[];
   if dispatchExpected
    timing=controlledTiming(row);jobTimingRows{end+1}=timing; %#ok<AGROW>
    verifyNativeJob(row,preflightCtx,initial,settings,timing);
   end
   verifyControlledDelivery(currentBeforeStep,C,row,initial,settings);
   assert(isempty(row.recovery_job) && ~row.supervisor.entered_recover && ...
    ~row.supervisor.recovery_search_pending && ~strcmp(char(row.supervisor.mode),'RECOVER'),'s27:RecoveryZeroViolated');
   currentStats=updateStats(currentStats,row,timing);assertBounds(counts,upper);
   if numel(rows)==50,flushChunk();end
   % Never call advance to obtain an empty terminal row or extra stateRow.
   if C.collision_pending,reason='COLLISION';break;
   elseif C.goal_pending,reason='ARRIVED';break;
   elseif C.next_tick>=1800,reason='TASK_TIMEOUT';break;end
  end
  if strcmp(reason,'RUNNING'),reason='TASK_TIMEOUT';end
  C.finished=true;C.termination=reason;flushChunk();
  caseResults{ci}=finishCase(caseId,reason,stopDetail,C,initial,currentStats,chunkPaths,m);
  counts.case_attempts_returned=counts.case_attempts_returned+1;
  save(fullfile(currentFolder,'terminal_state.mat'),'initial','C','counts','currentStats','caseResults','cfg','settings','-v7');
  writeJson(fullfile(currentFolder,'case_result.json'),caseResults{ci});
  counts.case_attempts_durable=counts.case_attempts_durable+1;durableCounts=counts;journal('case_finished');
  caseStarted=false;
  if wholeBatchStop,break;end
 end
 for ci=1:2
  if isempty(caseResults{ci}),caseResults{ci}=struct('case_id',labels{ci},'status','NOT_STARTED','reason','WHOLE_BATCH_STOP');end
 end
 summary.status='COMPLETED';if wholeBatchStop,summary.status='PARTIAL_RESOURCE_STOP';end
 summary.stage='source_after_and_archive';
 after=verifyFreeze(freeze.files);assert(isequaln(before,after) && strcmp(fileHash(freezePath),freezeSha),'s27:SourceChanged');
 p=profile('status');assert(strcmp(p.ProfilerStatus,'off'),'s27:ProfilerChanged');
 summary.source_unchanged=true;summary.freeze_self_unchanged=true;
 if ~wholeBatchStop
  summary.qualification_status='FAIL';
  if counts.case_attempts_durable==2 && counts.advance_returned==counts.intervals_durable && ...
   all(cellfun(@qualifiedArrival,caseResults))
   summary.qualification_status='PASS';
  end
 end
catch ME
 wholeBatchStop=true;summary.status='PARTIAL_ERROR';summary.stage_at_failure=summary.stage;
 summary.error_identifier=ME.identifier;summary.error_message=ME.message;summary.error_stack=getReport(ME,'extended','hyperlinks','off');
 summary.unreturned_advance=unreturnedAdvance;
 summary.unreturned_nested_work='UNKNOWN if a call began but did not return. Returned-row/source-derived counts are lower bounds; authorized maxima are separate. RAM is not equated with durable storage.';
 if caseStarted && ~isempty(currentFolder)
  try,flushChunk();catch flushME,summary.flush_error=getReport(flushME,'extended','hyperlinks','off');end
 end
 save(fullfile(runDir,'partial_failure_state.mat'),'C','initial','currentBeforeStep','rows','currentPreflight','preflightCtx', ...
  'cfg','settings','common','commonCase','externalRng','postdrawRng','jobTimingRows','counts','durableCounts','currentStats','caseResults','caseIndex', ...
  'startupDecision','startupResult','startupBefore','startupAfter','startupRngBefore','startupRngAfter','actualInputs','-v7');
 f=fopen(fullfile(runDir,'error_stack.txt'),'w');if f>=0,fprintf(f,'%s',summary.error_stack);fclose(f);end
 for ci=1:2
  if isempty(caseResults{ci})
   reason='WHOLE_BATCH_ERROR_BEFORE_CASE';if ci==caseIndex && caseStarted,reason='WHOLE_BATCH_ERROR_IN_CASE';end
   caseResults{ci}=struct('case_id',labels{ci},'status','NOT_COMPLETED','reason',reason);
  end
 end
 try,after=verifyFreeze(freeze.files);summary.source_unchanged=isequaln(before,after);summary.freeze_self_unchanged=strcmp(fileHash(freezePath),freezeSha);
 catch auditME,summary.source_after_error=getReport(auditME,'extended','hyperlinks','off');end
end
if ~isempty(after),writeJson(fullfile(runDir,'dependencies_after.json'),after);end
summary.cases=caseResults;summary.call_counts=counts;summary.authorized_upper=upper;summary.durable_counts_snapshot=durableCounts;
summary.count_semantics=struct('advance_begun','Actual p1b_engine step entry boundary', ...
 'native_dispatch_begun','Preflight passed scheduled advance entry; not proof of inner planner/selection entry if advance fails', ...
 'returned_work','Returned-row and execution-stat source-derived work; includes RAM until chunk commit', ...
 'durable_counts_snapshot','Last saved setup/startup/common checkpoint or committed chunk receipt; reconcile readable files after interrupted rename/receipt', ...
 'unreturned_work','UNKNOWN, bounded by separate authorized upper; no retry');
summary.setup_completed=setupCompleted;summary.typed_MAT_authoritative=true;summary.chunk_write_wall_s=writeWall;
summary.entry_elapsed_s=toc(entryClock);summary.finished_utc=utcNow();summary.whole_batch_stop=wholeBatchStop;
summary.interpretation='One qualification pair at common 9.6 m lookahead with fixed map, noise and 0.05 s logical delivery. Measured ready is a conditional calculation on these inputs.';
writeJson(fullfile(runDir,'summary.json'),summary);save(fullfile(runDir,'summary.mat'),'summary','counts','caseResults','-v7');
fprintf('S27_FINAL_STATUS=%s RETURNED_INTERVALS=%d DURABLE_INTERVALS=%d L1_RETURNED=%d\n',summary.status,counts.advance_returned,counts.intervals_durable,counts.total_l1_returned);
if ~strcmp(summary.status,'COMPLETED'),error('s27:Incomplete','Preserved partial evidence in %s',runDir);end

 function journal(event)
  record=struct('event',event,'utc',utcNow(),'entry_elapsed_s',toc(entryClock),'stage',summary.stage,'case_index',caseIndex, ...
   'counts',counts,'durable_counts_snapshot',durableCounts, ...
   'unreturned_advance',unreturnedAdvance,'typed_MAT_authoritative',true);
  if ~isempty(C),record.current_next_tick=C.next_tick;end
  writeJson(fullfile(runDir,'progress.json'),record);
 end
 function flushChunk()
  if isempty(rows),return;end
  timerWrite=tic;chunkIndex=chunkIndex+1;assert(chunkIndex<=36 && counts.chunks_durable<72,'s27:ChunkBound');
  path=fullfile(currentFolder,sprintf('chunk_%03d.mat',chunkIndex));
  chunk=struct('schema','s27-all-returned-rows-v1','case_id',initial.case_definition.global_method, ...
   'chunk_index',chunkIndex,'first_tick',rows{1}.tick,'last_tick',rows{end}.tick,'count',numel(rows), ...
   'rows',{rows},'job_timing',{jobTimingRows},'timing_contract_label','controlled_constant_0.05', ...
   'engine_timing_mode','replay','controlled_latency_s',.05,'actual_lookahead_m',initial.case_definition.scene.lookahead_m, ...
   'counts_before_durable_commit',counts,'C_after',C,'currentStats',currentStats, ...
   'intervals_durable_after_this_commit',counts.intervals_durable+numel(rows), ...
   'commit_rule','A fully readable chunk is authoritative; a hardkill after rename but before journal may require read-only reconciliation.');
  tmp=[path '.tmp.mat'];save(tmp,'chunk','-v7');[ok,msg]=movefile(tmp,path);assert(ok,'s27:ChunkCommit','%s',msg);
  counts.intervals_durable=counts.intervals_durable+numel(rows);counts.chunks_durable=counts.chunks_durable+1;durableCounts=counts;
  chunkPaths{end+1}=path;rows={};jobTimingRows={};writeWall=writeWall+toc(timerWrite);journal('chunk_committed');
 end
end

function launch=launchGate(base)
assert(strcmp(base,getenv('ASV27_APPROVED_BASE')) && strcmp(fullfile(base,'logs'),getenv('ASV27_APPROVED_LOGDIR')),'s27:LauncherDestination');
expected=getenv('ASV27_EXPECTED_TMPDIR');assert(~isempty(regexp(expected,'^/private/tmp/asv27\.[A-Za-z0-9]{6}$','once')),'s27:PrivateTempName');
actual=char(java.io.File(tempdir).getCanonicalPath());canonical=char(java.io.File(expected).getCanonicalPath());
assert(strcmp(expected,canonical) && strcmp(actual,expected),'s27:PrivateTempCanonical');
[code,stat]=system(sprintf('stat -f ''%%u %%Lp'' ''%s''',expected));
assert(code==0 && strcmp(strtrim(stat),'501 700'),'s27:PrivateTempOwnerMode');
assert(strcmp(getenv('ASV27_FREEZE'),fullfile(base,'audit','freeze.json')) && ...
 ~isempty(regexp(getenv('ASV27_FREEZE_SHA'),'^[0-9a-f]{64}$','once')),'s27:FreezeDestination');
assert(strcmp(which('run_s27_pair'),fullfile(base,'code','run_s27_pair.m')),'s27:RunnerEntry');
assert(isfolder(fullfile(base,'logs')),'s27:MissingLogDirectory');
launch=struct('base',base,'logs',fullfile(base,'logs'),'expected_private_tmpdir',expected, ...
 'matlab_tempdir',tempdir,'canonical_tempdir',actual,'uid',501,'permissions','0700', ...
 'entry',which('run_s27_pair'),'freeze',getenv('ASV27_FREEZE'));
end
function records=verifyFreeze(files)
records=struct('path',{},'sha256',{},'bytes',{});
for k=1:numel(files)
 info=dir(files(k).path);assert(isscalar(info) && ~info.isdir,'s27:FrozenSourceMissing','%s',files(k).path);
 h=fileHash(files(k).path);assert(strcmp(h,files(k).sha256) && info.bytes==files(k).bytes,'s27:FrozenSourceChanged','%s',files(k).path);
 records(k)=struct('path',files(k).path,'sha256',h,'bytes',info.bytes);
end
end
function resolution=pinResolution(base,repo,files)
stages=fileparts(base);priorPath=fullfile(stages,'S24','run_001','runtime_resolution.json');
requireFrozen(priorPath,files);prior=jsondecode(fileread(priorPath));
old={'q1_setup','p1b_setup','p1b_resolve_case','p1b_select','p1b_supervisor','p1b_predict','p0_reference_receiver', ...
 'p0_speed_controller','p003_dynamics_trace','p003_sweep_interval','p0_dynamics_step','otter_q_BOTH','LOSobserver','ssa','satlim','s04_candidate_to_reference'};
new={'p1b_engine','p1b_candidate','s10_batch_local_planner','s10_batch_reference_rollout','s13_forecast_receiver','s10_compare_science','s21_load_inputs','s21_preflight','s27_load_inputs','run_s27_pair'};
resolution=struct('name',{},'selected',{},'all_paths',{},'sha256',{});
for k=1:numel(old)
 p=which(old{k});ix=find(strcmp({prior.name},old{k}));assert(numel(ix)==1 && strcmp(p,prior(ix).selected) && strcmp(fileHash(p),prior(ix).sha256),'s27:HistoricalResolutionChanged','%s',old{k});
 requireFrozen(p,files);resolution(end+1)=resolutionRow(old{k},p); %#ok<AGROW>
end
dyn=fullfile(repo,'experiments','p1b_dynamic_selection_v1');
for name={'p1b_current','p1b_world_at','p1b_obstacles','p1b_crab_heading'}
 key=name{1};p=which(key);assert(strcmp(p,fullfile(dyn,[key '.m'])),'s27:SceneDependencyResolution','%s',key);
 requireFrozen(p,files);resolution(end+1)=resolutionRow(key,p); %#ok<AGROW>
end
for k=1:numel(new)
 p=which(new{k});assert(strcmp(p,fullfile(base,'code',[new{k} '.m'])),'s27:OverlayResolution','%s',new{k});
 requireFrozen(p,files);resolution(end+1)=resolutionRow(new{k},p); %#ok<AGROW>
end
assert(strcmp(which('q1_setup'),fullfile(repo,'experiments','q1_route_geometry_pilot_v1','q1_setup.m')),'s27:SetupResolution');
% Confirm that the engine and every original helper remain byte-identical.
for name={'p1b_engine.m','p1b_candidate.m','s10_batch_local_planner.m','s10_batch_reference_rollout.m', ...
  's13_forecast_receiver.m','s10_compare_science.m','s21_load_inputs.m','s21_preflight.m','protocol.json','cases.json'}
 here=fullfile(base,'code',name{1});there=fullfile(stages,'S24','code',name{1});
 requireFrozen(here,files);requireFrozen(there,files);
 assert(strcmp(fileHash(here),fileHash(there)),'s27:ProductionCopyChanged','%s',name{1});
end
end
function requireFrozen(path,files)
ix=find(strcmp({files.path},path));assert(numel(ix)==1 && strcmp(fileHash(path),files(ix).sha256),'s27:UnfrozenResolvedDependency','%s',path);
end
function r=resolutionRow(name,path)
r=struct('name',name,'selected',path,'all_paths',{which(name,'-all')},'sha256',fileHash(path));
end
function p=assertSpec(base,s,approval,files)
assert(strcmp(s.schema,'s27-approved-common-lookahead-execution-v1') && strcmp(s.approval_message,approval) && ...
 strcmp(s.contract_label,'controlled_constant_0.05') && strcmp(s.engine_timing_mode,'replay') && ...
 s.constant_latency_s==.05 && s.duration_s_each==90 && s.wall_limits.entry_cooperative_s==1500 && ...
 s.wall_limits.MATLAB_process_hard_s==1800 && ~s.wall_limits.goal_auto_stop && ...
 s.original_lookahead_m==30 && s.registered_lookahead_m==9.6 && ...
 strcmp(s.lookahead_binding,'C.case_definition.scene.lookahead_m') && ...
 strcmp(s.proposal_path,'sources/S27_common_lookahead_qualification_proposal.json') && ...
 strcmp(s.proposal_sha256,'cabf3cd6991d1c43ef0820d7808dae7090c2cb07b0c64d664f2aa2db19aa7701') && ...
 isequal(reshape(cellstr(string(s.case_order)),1,[]),{'ASTAR','THETA_STAR'}),'s27:InputSpecification');
pPath=fullfile(base,s.proposal_path);requireFrozen(pPath,files);
assert(strcmp(fileHash(pPath),s.proposal_sha256),'s27:ApprovedProposalHash');p=jsondecode(fileread(pPath));
aPath=fullfile(base,s.approval_path);requireFrozen(aPath,files);
assert(strcmp(fileHash(aPath),s.approval_sha256),'s27:ApprovedMessageHash');
assert(strcmp(p.schema,'S27_PROPOSAL_ONLY_S26_1') && strcmp(p.scientific_change.field,s.lookahead_binding) && ...
 p.scientific_change.from_m==30 && p.scientific_change.to_m==9.6 && p.batch.attempts_each==1 && ...
 p.batch.duration_each_s==90 && p.batch.entry_soft_s==1500 && p.batch.process_hard_s==1800 && ...
 isequal(reshape(cellstr(string(p.batch.run_order)),1,[]),{'ASTAR','THETA_STAR'}),'s27:ApprovedProposal');
for binding={s.s24_original_run_resolution,s.s24_original_runner,s.s24_original_freeze,s.s24_original_seal}
 b=binding{1};requireFrozen(b.path,files);assert(strcmp(fileHash(b.path),b.sha256),'s27:OriginalBinding');
end
end
function assertSceneMap(m)
assert(isequal(reshape(m.bounds_ne,1,[]),[-20,60,-20,60]) && isequal(reshape(m.start_ne,1,[]),[0,0]) && ...
 isequal(reshape(m.goal_ne,1,[]),[40,40]) && m.resolution_m==2 && abs(m.footprint_radius_m-hypot(1,.54))<=1e-14 && ...
 m.safety_margin_m==3 && abs(m.planning_inflation_m-(hypot(1,.54)+4))<=1e-14 && ...
 isequal(reshape(m.obstacles_ne,1,[]),[18,26,14,22]),'s27:FrozenSceneMismatch');
end
function assertRecoveryZero(C,s)
assert(s.control_dt_s==.05 && C.case_definition.scene.decision_dt_s==.5 && s.recovery.minimum_brake_s==1 && ...
 C.next_tick==0 && isempty(C.supervisor) && isempty(C.rx) && isempty(C.controller) && isempty(C.pending_jobs),'s27:RecoveryZeroPremise');
end
function assertPhysical(C,row)
assert(all(isfinite(C.x(:))) && all(isfinite(C.n(:))),'s27:NonfinitePhysicalState');
if ~isempty(C.controller)
 f=fieldnames(C.controller);for k=1:numel(f),v=C.controller.(f{k});assert(isnumeric(v) && all(isfinite(v(:))),'s27:NonfiniteControlState');end
end
if ~isempty(row)
 assert(all(isfinite(row.state_before(:))) && all(isfinite(row.state_after(:))) && all(isfinite(row.trace(:))),'s27:NonfinitePhysicalLog');
 for key={'n_cmd_left_radps','n_cmd_right_radps','target_speed_raw_mps','target_heading_rad'}
  assert(isfinite(row.action.(key{1})),'s27:NonfiniteControlCommand');
 end
 % +Inf geometric clearance and infeasible candidate costs are legitimate.
 assert(~isnan(row.geometry.min_clearance_m) && row.geometry.min_clearance_m~=-Inf,'s27:InvalidActualClearance');
end
end
function verifyProduction(d,require72)
assert(isstruct(d) && isscalar(d) && isfield(d,'s10_execution') && ~isfield(d,'s10_audit_all_rolls'),'s27:ProductionShape');
e=d.s10_execution;N=d.candidate_count;
assert(strcmp(e.backend,'BATCH') && e.original_l1_calls==0 && e.batch_kernel_calls==1,'s27:STOP_WHOLE_BATCH_UNREGISTERED_BACKEND');
assert(e.candidate_count==N && N>=1 && N<=84 && N==floor(N) && numel(d.s04_candidate_records)==N && ...
 ~e.audit_capture && ~e.audit_capture_available,'s27:CandidateShape');
assert(e.batch_stats.N==N && e.batch_stats.receiver_calls==240*N && e.batch_stats.math_updates==240*N && ...
 e.batch_stats.controller_batches==240 && e.batch_stats.body_batches==240 && e.batch_stats.planned_packets_per_column==24,'s27:CandidateWork');
if require72,assert(N==72,'s27:StartupN72');end
end
function verifyNativeJob(row,ctx,initial,s,timing)
b=row.job_bundles{1};assert(numel(b.jobs)==1 && strcmp(b.selection_log.policy,'FDWA'),'s27:JobCountOrPolicy');
j=b.jobs{1};d=j.decision;p=b.selected_packet;verifyProduction(d,false);
assert(strcmp(j.method,'L1') && j.context_isolation.context_unchanged && j.context_isolation.rng_unchanged,'s27:JobIsolation');
assert(j.candidate_scene.lookahead_m==9.6 && ...
 j.candidate_scene.lookahead_m==initial.case_definition.scene.lookahead_m,'s27:ResolvedCandidateLookahead');
assert(isequaln(ctx.public,j.execution_context.public) && isequaln(ctx.controller,j.execution_context.controller) && ...
 isequaln(ctx.rx,j.execution_context.rx) && strcmp(ctx.mode,j.execution_context.mode),'s27:PreflightActualContext');
[check,~]=s21_preflight(j.execution_context,ctx.cfg,ctx.cfg);assert(check.supported,'s27:ActualContextUnsupported');
assert(j.epoch==row.tick/10+1 && p.sequence==j.epoch && j.reference_core_identity.all_equal,'s27:JobEpochOrCoreIdentity');
core=d.reference_core;assert(isequal(size(p.t_ref),[1,31]) && isequaln(core.t_ref,p.t_ref) && ...
 isequaln(core.ur_ref,p.ur_ref) && isequaln(core.psi_ref,p.psi_ref) && ...
 isequaln(core.generated_from_time,p.generated_from_time) && isequaln(core.valid_until,p.valid_until),'s27:PacketCoreIdentity');
records=d.s04_candidate_records;best=Inf;winner=NaN;
for k=1:numel(records)
 if records(k).feasible && isfinite(records(k).score) && records(k).score<best,best=records(k).score;winner=k;end
end
if isfinite(winner)
 assert(d.s04_dwa_observation.winner_ordinal==records(winner).ordinal && isequaln(core,records(winner).core) && ...
  isequaln(d.s04_forecast.reference_core,core),'s27:FirstStrictMinimumIdentity');
else,assert(d.no_solution && strcmp(p.status,'failed'),'s27:FailureCoreStatus');end
selected=b.selection_log.selected_index;
if selected==0
 assert(strcmp(p.planner_id,'NONE') && strcmp(p.reference_id,sprintf('P1B-NO-CHOICE-%06d',j.epoch)) && ...
  strcmp(p.status,'failed') && strcmp(p.failure_reason,'selector_no_safe_candidate'),'s27:NoChoiceWrapperIdentity');
else,assert(selected==1 && strcmp(p.planner_id,'L1') && strcmp(p.reference_id,sprintf('P1B-L1-%06d',j.epoch)),'s27:PacketIdentity');end
logicalReady=timing.logical_ready_s;
assert(abs(p.ready_time-logicalReady)<=1e-12 && abs(b.ready_time-logicalReady)<=1e-12 && ...
 abs(j.shared_ready_time-logicalReady)<=1e-12 && abs(b.selection_log.ready_time-logicalReady)<=1e-12 && ...
 b.selection_log.generated_from_time==j.generated_from_time && ...
 b.generated_from_time==j.generated_from_time && j.generated_from_time==row.t_s, ...
 's27:ControlledReadyPropagation');
for q=1:numel(b.candidate_packets)
 assert(abs(b.candidate_packets{q}.ready_time-logicalReady)<=1e-12,'s27:ControlledCandidateReady');
end
assert(isequal(size(b.candidate_packets),[1,1]) && initial.case_definition.injected_latency_s==.05 && ...
 s.compute_budget_s==.5,'s27:ControlledBudgetInputs');
end
function c=zeroCounts()
c=struct('setup_begun',0,'setup_returned',0,'startup_l1_begun',0,'startup_l1_returned',0, ...
 'common_init_begun',0,'common_init_returned',0,'new_noise_draws',0,'new_global_searches',0, ...
 'new_route_metrics_calls',0,'rng_state_restores',0,'case_attempts_begun',0,'case_attempts_returned',0, ...
 'case_attempts_durable',0,'advance_begun',0,'advance_returned',0, ...
 'native_dispatch_begun',0,'native_jobs_returned',0,'fdwa_selections_returned',0, ...
 'total_l1_returned',0,'scalar_l1_calls_reported',0,'batch_kernels_reported',0,'candidate_columns_reported',0, ...
 'candidate_private_receiver_calls_reported',0,'candidate_math_updates_reported',0, ...
 'candidate_controller_batches_reported',0,'candidate_body_batches_reported',0, ...
 'candidate_legacy_kinematic_updates_source_derived',0,'candidate_cost_samples_source_derived',0, ...
 'fullplant_intervals_completed',0,'actual_controller_calls_source_derived',0,'native_receiver_calls_source_derived',0, ...
 'guard_forecast_calls_source_derived',0,'guard_reduced_updates_source_derived',0, ...
 'recovery_search_forecasts',0,'recovery_pending_forecasts',0,'recovery_recheck_forecasts',0, ...
 'BOTH_rhs_from_completed_intervals',0,'setup_BOTH_rhs_on_return',0,'setup_archived_otter_on_return',0, ...
 'external_speed_controller_source_derived',0,'chunks_durable',0,'intervals_durable',0);
end
function c=addDecisionWork(c,d)
c.total_l1_returned=c.total_l1_returned+1;
if ~isfield(d,'s10_execution'),return;end
e=d.s10_execution;
if isfield(e,'original_l1_calls'),c.scalar_l1_calls_reported=c.scalar_l1_calls_reported+e.original_l1_calls;end
if isfield(e,'batch_kernel_calls'),c.batch_kernels_reported=c.batch_kernels_reported+e.batch_kernel_calls;end
if isfield(e,'batch_stats')
 st=e.batch_stats;c.candidate_columns_reported=c.candidate_columns_reported+st.N;
 c.candidate_private_receiver_calls_reported=c.candidate_private_receiver_calls_reported+st.receiver_calls;
 c.candidate_math_updates_reported=c.candidate_math_updates_reported+st.math_updates;
 c.candidate_controller_batches_reported=c.candidate_controller_batches_reported+st.controller_batches;
 c.candidate_body_batches_reported=c.candidate_body_batches_reported+st.body_batches;
 c.candidate_legacy_kinematic_updates_source_derived=c.candidate_legacy_kinematic_updates_source_derived+42;
 c.candidate_cost_samples_source_derived=c.candidate_cost_samples_source_derived+30*st.N;
end
end
function c=addReturnedWork(c,r)
for k=1:numel(r.job_bundles)
 b=r.job_bundles{k};c.fdwa_selections_returned=c.fdwa_selections_returned+1;
 for j=1:numel(b.jobs),c.native_jobs_returned=c.native_jobs_returned+1;c=addDecisionWork(c,b.jobs{j}.decision);end
end
if ~isempty(fieldnames(r.action)) && r.action.interval_completed==1
 c.fullplant_intervals_completed=c.fullplant_intervals_completed+1;
 c.actual_controller_calls_source_derived=c.actual_controller_calls_source_derived+1;
 c.native_receiver_calls_source_derived=c.native_receiver_calls_source_derived+2;
 c.guard_forecast_calls_source_derived=c.guard_forecast_calls_source_derived+2;
 c.guard_reduced_updates_source_derived=c.guard_reduced_updates_source_derived+200;
 c.BOTH_rhs_from_completed_intervals=c.BOTH_rhs_from_completed_intervals+34;
 c.external_speed_controller_source_derived=c.external_speed_controller_source_derived+201;
end
if c.setup_returned==1,c.setup_BOTH_rhs_on_return=1;c.setup_archived_otter_on_return=1;end
end
function u=authorizedUpper()
u=struct('MATLAB_processes',1,'setup',1,'startup',1,'common_init',0,'new_noise_draws',0,'new_global_searches',0, ...
 'new_route_metrics_calls',0,'case_attempts',2,'native_jobs',360,'total_L1',361, ...
 'kernel_calls',361,'candidate_columns',30312,'candidate_private_receiver_calls',7274880,'candidate_math_updates',7274880, ...
 'candidate_controller_batches',86640,'candidate_body_batches',86640,'legacy_kinematic_updates',15162,'cost_samples',909360, ...
 'fullplant_intervals',3600,'actual_controller',3600,'native_receiver',7200,'guard_forecasts',7200,'guard_reduced_updates',720000, ...
 'recovery_all',0,'external_speed_controller',723600,'BOTH_rhs',122401,'archived_otter',1,'reused_noise_values',7216,'chunks',72);
end
function assertBounds(c,u)
assert(c.setup_begun<=u.setup && c.startup_l1_begun<=u.startup && c.common_init_begun<=u.common_init && ...
 c.common_init_returned==0 && c.new_noise_draws==0 && c.new_global_searches==0 && ...
 c.new_route_metrics_calls==0 && c.rng_state_restores<=3 && ...
 c.advance_begun<=u.fullplant_intervals && c.native_dispatch_begun<=u.native_jobs && c.native_jobs_returned<=u.native_jobs && ...
 c.total_l1_returned<=u.total_L1 && c.scalar_l1_calls_reported==0 && c.batch_kernels_reported<=u.kernel_calls && ...
 c.candidate_columns_reported<=u.candidate_columns && c.candidate_math_updates_reported<=u.candidate_math_updates && ...
 c.guard_reduced_updates_source_derived<=u.guard_reduced_updates && c.external_speed_controller_source_derived<=u.external_speed_controller && ...
 c.BOTH_rhs_from_completed_intervals+c.setup_BOTH_rhs_on_return<=u.BOTH_rhs,'s27:ReturnedLedgerBound');
end
function st=zeroStats(initial)
st=struct('intervals',0,'jobs',0,'actual_length_m',0,'minimum_swept_net_clearance_m',Inf,'margin_violation_s',0, ...
 'first_contact_time_s',NaN,'shaft_effort_proxy_rad3_sneg2',0,'planner_fresh_intervals',0,'fallback_intervals',0,'safety_override_intervals',0, ...
 'control_deadline_exceeded_intervals',0,'candidate_overbudget_jobs',0, ...
 'native_receiver_tolerance_overbudget_jobs',0,'last_receive_check_s',NaN,'last_job_s',NaN, ...
 'terminal_state_time_s',0,'initial_NE',initial.x(7:8)');
end
function st=updateStats(st,r,timing)
a=r.action;st.intervals=st.intervals+1;st.jobs=st.jobs+numel(r.job_bundles);
st.actual_length_m=st.actual_length_m+sum(vecnorm(diff(r.trace(:,10:11)),2,2));
st.minimum_swept_net_clearance_m=min(st.minimum_swept_net_clearance_m,r.geometry.min_clearance_m);
st.margin_violation_s=st.margin_violation_s+r.geometry.margin_violation_s;
if isfinite(r.geometry.first_contact_time_s)
 if ~isfinite(st.first_contact_time_s),st.first_contact_time_s=r.geometry.first_contact_time_s;
 else,st.first_contact_time_s=min(st.first_contact_time_s,r.geometry.first_contact_time_s);end
end
st.shaft_effort_proxy_rad3_sneg2=st.shaft_effort_proxy_rad3_sneg2+ ...
 (abs(a.n_actual_left_radps)^3+abs(a.n_actual_right_radps)^3)*.05;
if ~strcmp(char(a.execution_source),'PLANNER') || ~strcmp(char(a.supervisor_mode),'TRACK')
 st.safety_override_intervals=st.safety_override_intervals+1;
elseif a.fallback || ~a.reference_fresh,st.fallback_intervals=st.fallback_intervals+1;
else,st.planner_fresh_intervals=st.planner_fresh_intervals+1;end
st.control_deadline_exceeded_intervals=st.control_deadline_exceeded_intervals+a.control_deadline_exceeded;
st.last_receive_check_s=r.t_s;st.terminal_state_time_s=r.state_after(2);
if ~isempty(r.job_bundles)
 b=r.job_bundles{1};st.last_job_s=b.generated_from_time;
 st.candidate_overbudget_jobs=st.candidate_overbudget_jobs+double(~timing.strict_measured_budget_pass);
 st.native_receiver_tolerance_overbudget_jobs=st.native_receiver_tolerance_overbudget_jobs+ ...
  double(~timing.native_receiver_tolerance_budget_pass);
end
end
function out=finishCase(id,reason,detail,C,initial,st,paths,globalResult)
assert(st.intervals==C.next_tick && st.jobs==C.global_epoch,'s27:CaseReturnedLedger');
if strcmp(reason,'TASK_TIMEOUT')
 assert(C.next_tick==1800 && st.jobs==180 && abs(st.last_job_s-89.5)<=1e-12 && ...
  abs(st.last_receive_check_s-89.95)<=1e-12 && abs(st.terminal_state_time_s-90)<=1e-12,'s27:FinalBoundary');
end
out=struct('case_id',id,'status',reason,'stop_detail',detail,'summary_metrics',st, ...
 'completed_intervals',C.next_tick,'native_epochs',C.global_epoch,'terminal_NE',C.x(7:8)', ...
 'terminal_heading_rad',C.x(12),'distance_to_exact_goal_m',norm(C.x(7:8)'-[40,40]), ...
 'arrived',C.goal_pending,'collision',C.collision_pending,'pending_bundle_count',numel(C.pending_jobs), ...
 'pending_saved_unchanged',true,'chunks',{paths},'global_result',globalResult, ...
 'initial_route_NE',initial.case_definition.scene.global_route_ne,'maxT_s',90,'goal_radius_m',5, ...
 'endpoint_rule','R0/R1 exact [40,40]; X stops at5m goal region or explicit stop. No length retention ratio.', ...
 'clearance_rule','Circumcircle swept along stored4substep straight pieces per interval against physical rectangle and bounds; not exact hull/nonlinearODE guarantee.', ...
 'effort_rule','Left sampled sum((abs(nL)^3+abs(nR)^3)*.05), effort proxy only; not fuel/electric energy.', ...
 'timing_contract_label','controlled_constant_0.05','engine_timing_mode','replay','controlled_latency_s',.05, ...
 'actual_lookahead_m',initial.case_definition.scene.lookahead_m, ...
 'no_additional_terminal_engine_step',true);
end
function checkSoft(clock)
assert(toc(clock)<1500,'s27:EntrySoftWall','Cooperative resource limit reached at call boundary; no new model call.');
end
function s=inputSnapshot(ctx,scene,obs,objects,previous)
s=struct('ctx',ctx,'scene',scene,'obs',obs,'objects',{objects},'previous',previous);
end
function t=utcNow(),t=char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ss.SSS''Z'''));end
function h=fileHash(path)
f=fopen(path,'rb');assert(f>=0,'s27:SourceRead','%s',path);cl=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);
h=lower(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
function writeJson(path,value)
% MAT remains authoritative. Explicit typed projection for every nonfinite array;
% never silently turn Inf/NaN into JSON null or a finite zero and never round.
value=jsonSafe(value);text=jsonencode(value,'PrettyPrint',true);tmp=[path '.tmp'];
f=fopen(tmp,'w','n','UTF-8');assert(f>=0,'s27:WriteJson','%s',path);
cl=onCleanup(@()fclose(f));fprintf(f,'%s\n',text);clear cl;
[ok,msg]=movefile(tmp,path);assert(ok,'s27:JsonCommit','%s',msg);
end
function out=jsonSafe(v)
if isstruct(v)
 out=v;names=fieldnames(v);for k=1:numel(v),for j=1:numel(names),out(k).(names{j})=jsonSafe(v(k).(names{j}));end,end
elseif iscell(v)
 out=cell(size(v));for k=1:numel(v),out{k}=jsonSafe(v{k});end
elseif isnumeric(v) && any(~isfinite(v(:)))
 values=cell(size(v));for k=1:numel(v)
  if isnan(v(k)),values{k}='NaN';elseif v(k)==Inf,values{k}='+Inf';elseif v(k)==-Inf,values{k}='-Inf';else,values{k}=v(k);end
 end
 out=struct('projection_type','MATLAB_numeric_with_nonfinite','matlab_class',class(v),'size',size(v), ...
  'column_major_values',{reshape(values,1,[])},'authoritative','Corresponding typed MAT');
else,out=v;
end
end

function t=controlledTiming(row)
% These annotations are computed only after engine return. Raw R is unchanged.
b=row.job_bundles{1};j=b.jobs{1};g=j.generated_from_time;
candidate=j.actual_cpu_s;selector=b.selection_log.actual_cpu_s;
outer=j.outer_total_s;total=candidate+selector;
assert(all(isfinite([g,candidate,selector,outer,total])) && min([candidate,selector,outer])>=0, ...
 's27:MeasuredTimingValues');
assert(candidate==b.selection_log.total_candidate_cpu_s && candidate==row.action.candidate_cpu_s && ...
 selector==row.action.selection_cpu_s,'s27:MeasuredTimingBinding');
nativeReady=g+max(total,.05);logicalReady=g+.05;
tolerancePass=nativeReady-g<=.5+1e-10;
t=struct('tick',row.tick,'epoch',b.epoch,'sequence',b.selected_packet.sequence,'generation_s',g, ...
 'packet_id',b.selected_packet.reference_id,'candidate_count',j.decision.candidate_count, ...
 'actual_candidate_s',candidate,'actual_selector_s',selector,'actual_candidate_plus_selector_s',total, ...
 'candidate_adapter_outer_s',outer,'native_measured_ready_s',nativeReady, ...
 'strict_measured_budget_pass',total<=.5,'native_receiver_tolerance_budget_pass',tolerancePass, ...
 'measured_compute_budget_pass',tolerancePass,'logical_ready_s',logicalReady, ...
 'logical_compute_budget_pass',logicalReady-g<=.5+1e-10,'actual_bundle_ready_s',b.ready_time, ...
 'actual_selected_packet_ready_s',b.selected_packet.ready_time, ...
 'actual_job_shared_ready_s',j.shared_ready_time,'actual_selection_ready_s',b.selection_log.ready_time, ...
 'actual_candidate_packet_ready_s',b.candidate_packets{1}.ready_time, ...
 'control_deadline_exceeded',row.action.control_deadline_exceeded, ...
 'timing_contract_label','controlled_constant_0.05','engine_timing_mode','replay');
end

function verifyControlledDelivery(previous,C,row,initial,s)
% Reconcile saved queue/receiver fields using the engine's original predicates.
assert(strcmp(C.timing_mode,'replay') && isequal(C.delivery_latency_tape,initial.delivery_latency_tape) && ...
 isequal(size(C.delivery_latency_tape),[184,1]) && all(C.delivery_latency_tape==.05) && ...
 C.timing_tape_index==C.global_epoch+1 && C.global_epoch==previous.global_epoch+numel(row.job_bundles) && ...
 isequaln(C.case_definition,initial.case_definition),'s27:TimingTapeOrCaseMutation');
keep={};incoming={};epochs=previous.delivered_epochs;
for k=1:numel(previous.pending_jobs)
 b=previous.pending_jobs{k};p=b.selected_packet;g=b.generated_from_time;
 assert(abs(b.ready_time-(g+.05))<=1e-12 && abs(p.ready_time-b.ready_time)<=1e-12 && ...
  p.generated_from_time==g,'s27:PendingReadyBinding');
 if b.ready_time<=row.t_s+1e-10
  index=numel(incoming)+1;
  assert(index<=numel(row.received),'s27:DuePacketNotRecorded');
  a=row.received{index};
  assert(a.epoch==b.epoch && a.arrival_time==row.t_s && ...
   abs(a.arrival_time-b.ready_time)<=1e-10 && a.scheduled_arrival_time==b.ready_time && ...
   strcmp(a.policy,'FDWA') && isequaln(a.packet,p) && isequaln(a.selection_log,b.selection_log), ...
   's27:ControlledActualReception');
  seq=-1;generation=-Inf;
  if ~isempty(previous.rx),seq=previous.rx.sequence_watermark;generation=previous.rx.generation_watermark;end
  eligible=p.sequence>seq && p.generated_from_time>=generation && p.ready_time<=row.t_s+1e-10 && ...
   row.t_s<p.valid_until && p.ready_time-p.generated_from_time<=s.compute_budget_s+1e-10 && ...
   row.t_s-p.generated_from_time<=s.max_arrival_age_s+1e-10;
  assert(logical(a.causally_eligible)==eligible,'s27:ControlledCausalReceipt');
  incoming{end+1}=p;epochs(end+1)=b.epoch; %#ok<AGROW>
 else
  keep{end+1}=b; %#ok<AGROW>
 end
end
assert(numel(row.received)==numel(incoming) && isequaln(C.delivered_epochs,epochs),'s27:ReceivedEpochIdentity');
for k=1:numel(row.job_bundles)
 b=row.job_bundles{k};j=b.jobs{1};
 assert(isequaln(j.execution_context.incoming,incoming),'s27:PlannerIncomingIdentity');
 keep{end+1}=b; %#ok<AGROW>
end
assert(isequaln(C.pending_jobs,keep),'s27:PendingQueueIdentity');
for k=1:numel(row.receiver_events)
 e=row.receiver_events(k);
 if isfinite(e.sequence)
  assert(isfinite(e.generated_from_time) && isfinite(e.ready_time) && ...
   abs(e.ready_time-(e.generated_from_time+.05))<=1e-12,'s27:ActualEventReadyBinding');
 end
end
if row.action.reference_fresh
 a=row.action;
 assert(isfinite(a.reference_generated_time_s) && isfinite(a.reference_ready_time_s) && ...
  abs(a.reference_ready_time_s-(a.reference_generated_time_s+.05))<=1e-12,'s27:ActiveReferenceReadyBinding');
end
end

function yes=qualifiedArrival(r)
yes=isstruct(r) && isfield(r,'status') && strcmp(r.status,'ARRIVED') && r.arrived && ~r.collision && ...
 r.distance_to_exact_goal_m<=5 && r.completed_intervals>0 && ~isempty(r.chunks);
end
