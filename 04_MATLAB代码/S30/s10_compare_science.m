function result=s10_compare_science(referenceDecision,currentDecision,referenceRolls,currentRolls)
% Complete stored-science comparison only. Never evaluates a model or search.
% Only the three approved TOP-LEVEL non-science fields are removed.
if nargin<3,referenceRolls={};end
if nargin<4,currentRolls={};end
referenceDecision=cleanTop(referenceDecision);
currentDecision=cleanTop(currentDecision);
stats=struct('numeric_values',0,'finite_pairs',0,'nan_pairs',0,'positive_inf_pairs',0, ...
 'negative_inf_pairs',0,'nonfinite_class_mismatches',0,'difference_groups',0, ...
 'max_absolute_error',0,'max_tolerance_ratio',0,'max_state_time_abs_error_s',0, ...
 'max_state_body_abs_error',0,'max_reference_abs_error',0,'max_reference_shaft_abs_error',0, ...
 'max_core_direct_angle_error_rad',0,'max_core_wrapped_angle_error_rad',0);
first=struct('found',false,'ordinal',NaN,'time',NaN,'field','','left',[], ...
 'right',[],'tolerance',[],'reason','','wrapped_difference',[]);
refMin=firstMinimum(referenceDecision);curMin=firstMinimum(currentDecision);
origin=decisionOrigin(referenceDecision);
walk(referenceDecision,currentDecision,'decision',NaN,origin);
minimumPass=refMin.valid && curMin.valid && refMin.found==curMin.found;
if minimumPass && refMin.found
 minimumPass=refMin.ordinal==curMin.ordinal && refMin.observation_consistent && curMin.observation_consistent;
elseif minimumPass
 minimumPass=refMin.observation_consistent && curMin.observation_consistent;
end
if ~minimumPass
 note(NaN,origin,'decision.first_strict_finite_feasible_minimum',refMin,curMin,0,'first_minimum_or_reported_winner_mismatch',[]);
end
decisionPass=stats.difference_groups==0;
beforeRolls=stats.difference_groups;perOrdinal={};
bothEmpty=iscell(referenceRolls) && iscell(currentRolls) && isempty(referenceRolls) && isempty(currentRolls);
if ~bothEmpty
 if ~iscell(referenceRolls) || ~iscell(currentRolls)
  note(NaN,NaN,'rolls.class',class(referenceRolls),class(currentRolls),[],'rolls_must_be_cells',[]);
 elseif ~isequal(size(referenceRolls),size(currentRolls)) || size(referenceRolls,1)~=1
  note(NaN,NaN,'rolls.shape',size(referenceRolls),size(currentRolls),[],'rolls_must_be_matching_1_by_N',[]);
 else
  N=numel(referenceRolls);
  if N~=refMin.candidate_count || N~=curMin.candidate_count
   note(NaN,NaN,'rolls.candidate_count',[N,refMin.candidate_count],[N,curMin.candidate_count],0,'full_roll_count_must_match_decision_records',[]);
  end
  perOrdinal=cell(1,N);
  for ordinal=1:N
   countBefore=stats.difference_groups;
   a=referenceRolls{ordinal};b=currentRolls{ordinal};
   if ~isstruct(a) || ~isscalar(a) || ~isstruct(b) || ~isscalar(b)
    note(ordinal,NaN,sprintf('rolls{%d}',ordinal),class(a),class(b),[],'each_roll_must_be_scalar_struct',[]);
   else
    checkRollShape(a,b,ordinal);
    walk(a,b,sprintf('rolls{%d}',ordinal),ordinal,rollOrigin(a));
   end
   perOrdinal{ordinal}=struct('ordinal',ordinal,'passed',stats.difference_groups==countBefore);
  end
 end
end
rollsPass=stats.difference_groups==beforeRolls;
result=struct('passed',decisionPass && rollsPass,'decision_pass',decisionPass, ...
 'rolls_pass',rollsPass,'roll_comparison_requested',~bothEmpty, ...
 'first_difference',first,'max_stats',stats, ...
 'first_minimum_reference',refMin,'first_minimum_current',curMin, ...
 'first_minimum_pass',minimumPass,'per_ordinal',{perOrdinal}, ...
 'core_angular_checks',struct('maximum_direct_error_rad',stats.max_core_direct_angle_error_rad, ...
 'maximum_wrapped_error_rad',stats.max_core_wrapped_angle_error_rad, ...
 'gate','Direct absolute 1e-12; wrapped error is reported separately, never substituted for internal-angle equality.'), ...
 'removed_top_level_fields',{{'s10_audit_all_rolls','s10_execution','decision_time_s'}}, ...
 'scope','All existing fields, class, shape, text, traces, events, final public/controller/rx and core are compared. No unrecorded intermediate integrals or receiver state is manufactured.');

 function checkRollShape(a,b,ordinal)
  required={'state_trace','reference_trace','events','final_public','final_controller','final_rx','reference_core'};
  if ~all(isfield(a,required)) || ~all(isfield(b,required))
   note(ordinal,NaN,sprintf('rolls{%d}.required_fields',ordinal),fieldnames(a),fieldnames(b),[],'missing_full_roll_field',[]);
   return
  end
  if ~isequal(size(a.state_trace),[241,9]) || ~isequal(size(b.state_trace),[241,9])
   note(ordinal,NaN,sprintf('rolls{%d}.state_trace.shape',ordinal),size(a.state_trace),size(b.state_trace),[],'expected_241_by_9',[]);
  end
  if ~isequal(size(a.reference_trace),[240,12]) || ~isequal(size(b.reference_trace),[240,12])
   note(ordinal,NaN,sprintf('rolls{%d}.reference_trace.shape',ordinal),size(a.reference_trace),size(b.reference_trace),[],'expected_240_by_12',[]);
  end
  if ~isnumeric(a.state_trace) || ~isnumeric(b.state_trace) || ~all(isfinite(a.state_trace(:))) || ~all(isfinite(b.state_trace(:)))
   note(ordinal,NaN,sprintf('rolls{%d}.state_trace.finite',ordinal), ...
    isnumeric(a.state_trace) && all(isfinite(a.state_trace(:))),isnumeric(b.state_trace) && all(isfinite(b.state_trace(:))),0,'frozen_S08_finite_state_requirement',[]);
  end
 end

 function walk(a,b,path,ordinal,time)
  if ~strcmp(class(a),class(b))
   note(ordinal,time,[path '.class'],class(a),class(b),[],'class_mismatch',[]);return
  end
  if ~isequal(size(a),size(b))
   note(ordinal,time,[path '.shape'],size(a),size(b),[],'shape_mismatch',[]);return
  end
  if isstruct(a)
   fa=fieldnames(a);fb=fieldnames(b);
   if ~isequal(fa,fb)
    note(ordinal,time,[path '.fields'],fa,fb,[],'field_names_or_order_mismatch',[]);return
   end
   for si=1:numel(a)
    localOrdinal=ordinal;localTime=time;elementPath=path;
    if numel(a)>1,elementPath=sprintf('%s(%d)',path,si);end
    if endsWith(path,'.s04_candidate_records') && isfield(a(si),'ordinal') && finiteScalar(a(si).ordinal)
     localOrdinal=a(si).ordinal;
    end
    if isfield(a(si),'t_s') && finiteScalar(a(si).t_s),localTime=a(si).t_s;
    elseif isfield(a(si),'last_time_s') && finiteScalar(a(si).last_time_s),localTime=a(si).last_time_s;end
    for fi=1:numel(fa)
     field=fa{fi};childOrdinal=localOrdinal;childTime=localTime;
     if strcmp(field,'s04_forecast') && refMin.found,childOrdinal=refMin.ordinal;end
     if strcmp(field,'final_controller') && isfield(a(si),'final_public') && isfield(a(si).final_public,'t_s')
      childTime=a(si).final_public.t_s;
     end
     walk(a(si).(field),b(si).(field),[elementPath '.' field],childOrdinal,childTime);
    end
   end
  elseif iscell(a)
   for ci=1:numel(a),walk(a{ci},b{ci},sprintf('%s{%d}',path,ci),ordinal,time);end
  elseif isnumeric(a)
   compareNumeric(a,b,path,ordinal,time);
  elseif islogical(a)
   if ~isequal(a,b),note(ordinal,time,path,a,b,0,'logical_mismatch',[]);end
  elseif ischar(a) || isstring(a)
   if ~isequaln(a,b),note(ordinal,time,path,a,b,0,'text_mismatch',[]);end
  else
   if ~isequaln(a,b),note(ordinal,time,path,a,b,0,'other_exact_mismatch',[]);end
  end
 end

 function compareNumeric(a,b,path,ordinal,time)
  stats.numeric_values=stats.numeric_values+numel(a);
  if isempty(a),return;end
  if ~isequal(isreal(a),isreal(b)) || ~isreal(a) || ~isreal(b)
   if ~isequaln(a,b),note(ordinal,time,path,a,b,[],'complex_representation_mismatch',[]);end
   return
  end
  if isinteger(a)
   if ~isequal(a,b),note(ordinal,time,path,a,b,0,'integer_mismatch',[]);end
   return
  end
  aa=double(a);bb=double(b);
  nanPair=isnan(aa) & isnan(bb);
  posPair=isinf(aa) & aa>0 & isinf(bb) & bb>0;
  negPair=isinf(aa) & aa<0 & isinf(bb) & bb<0;
  finite=isfinite(aa) & isfinite(bb);
  specialBad=~(nanPair | posPair | negPair | finite);
  stats.nan_pairs=stats.nan_pairs+sum(nanPair(:));
  stats.positive_inf_pairs=stats.positive_inf_pairs+sum(posPair(:));
  stats.negative_inf_pairs=stats.negative_inf_pairs+sum(negPair(:));
  stats.finite_pairs=stats.finite_pairs+sum(finite(:));
  stats.nonfinite_class_mismatches=stats.nonfinite_class_mismatches+sum(specialBad(:));
  err=zeros(size(aa));err(finite)=abs(aa(finite)-bb(finite));
  [tol,kind]=tolerances(aa,bb,finite,path);
  finiteBad=finite & err>tol;
  bad=specialBad | finiteBad;
  if any(finite(:))
   stats.max_absolute_error=max(stats.max_absolute_error,max(err(finite)));
   ratio=zeros(size(err));positiveTol=finite & tol>0;
   ratio(positiveTol)=err(positiveTol)./tol(positiveTol);
   ratio(finite & tol==0 & err>0)=Inf;
   stats.max_tolerance_ratio=max(stats.max_tolerance_ratio,max(ratio(finite)));
  end
  if strcmp(kind,'state')
   stats.max_state_time_abs_error_s=max(stats.max_state_time_abs_error_s,max(err(:,1)));
   body=err(:,2:end);stats.max_state_body_abs_error=max(stats.max_state_body_abs_error,max(body(:)));
  elseif strcmp(kind,'reference')
   refs=err(:,1:7);shaft=err(:,8:9);
   stats.max_reference_abs_error=max(stats.max_reference_abs_error,max(refs(:)));
   stats.max_reference_shaft_abs_error=max(stats.max_reference_shaft_abs_error,max(shaft(:)));
  end
  leaf=leafName(path);
  coreAngle=isCore(path) && (contains(leaf,'psi') || contains(leaf,'heading'));
  wrapped=[];
  if coreAngle
   wrapped=zeros(size(aa));wrapped(finite)=abs(atan2(sin(aa(finite)-bb(finite)),cos(aa(finite)-bb(finite))));
   if any(finite(:))
    stats.max_core_direct_angle_error_rad=max(stats.max_core_direct_angle_error_rad,max(err(finite)));
    stats.max_core_wrapped_angle_error_rad=max(stats.max_core_wrapped_angle_error_rad,max(wrapped(finite)));
   end
  end
  if any(bad(:))
   linear=firstNumericIndex(bad,kind);
   [row,column]=ind2sub(size(a),linear);
   localTime=time;
   if any(strcmp(kind,{'state','reference'})) && size(aa,2)>=1 && isfinite(aa(row,1)),localTime=aa(row,1);end
   localWrapped=[];if coreAngle && finite(linear),localWrapped=wrapped(linear);end
   localTol=tol(linear);reason='finite_tolerance_exceeded';
   if specialBad(linear),localTol=[];reason='finite_nan_positive_inf_negative_inf_class_mismatch';end
   note(ordinal,localTime,sprintf('%s(%d,%d)',path,row,column),a(linear),b(linear),localTol,reason,localWrapped);
  end
 end

 function note(ordinal,time,field,left,right,tolerance,reason,wrapped)
  stats.difference_groups=stats.difference_groups+1;
  candidate=struct('found',true,'ordinal',ordinal,'time',time,'field',field, ...
   'left',left,'right',right,'tolerance',tolerance,'reason',reason,'wrapped_difference',wrapped);
  if ~first.found || earlier(candidate,first),first=candidate;end
 end
end

function d=cleanTop(d)
if isstruct(d) && isscalar(d)
 for name={'s10_audit_all_rolls','s10_execution','decision_time_s'}
  if isfield(d,name{1}),d=rmfield(d,name{1});end
 end
end
end
function out=firstMinimum(d)
out=struct('valid',false,'candidate_count',0,'found',false,'index',NaN,'ordinal',NaN, ...
 'score',Inf,'observation_consistent',false);
if ~isstruct(d) || ~isscalar(d) || ~isfield(d,'s04_candidate_records') || ~isstruct(d.s04_candidate_records),return;end
records=d.s04_candidate_records;out.candidate_count=numel(records);out.valid=true;
best=Inf;
for k=1:numel(records)
 r=records(k);
 if ~all(isfield(r,{'ordinal','feasible','score'})) || ~isscalar(r.feasible) || ~finiteScalar(r.ordinal)
  out.valid=false;return
 end
 if logical(r.feasible) && finiteScalar(r.score) && r.score<best
  best=r.score;out.found=true;out.index=k;out.ordinal=r.ordinal;out.score=r.score;
 end
end
if ~isfield(d,'s04_dwa_observation') || ~isstruct(d.s04_dwa_observation),return;end
ob=d.s04_dwa_observation;
if ~all(isfield(ob,{'winner_found','winner_ordinal'})) || ~isscalar(ob.winner_found),return;end
if out.found
 out.observation_consistent=logical(ob.winner_found) && finiteScalar(ob.winner_ordinal) && ob.winner_ordinal==out.ordinal;
else
 out.observation_consistent=~logical(ob.winner_found) && isfield(d,'no_solution') && logical(d.no_solution);
end
end
function yes=finiteScalar(x)
yes=isnumeric(x) && isreal(x) && isscalar(x) && isfinite(x);
end
function t=decisionOrigin(d)
t=NaN;
if isstruct(d) && isscalar(d) && isfield(d,'s04_dwa_observation') && ...
 isfield(d.s04_dwa_observation,'origin_t_s') && finiteScalar(d.s04_dwa_observation.origin_t_s)
 t=d.s04_dwa_observation.origin_t_s;
end
end
function t=rollOrigin(r)
t=NaN;
if isstruct(r) && isscalar(r) && isfield(r,'state_trace') && isnumeric(r.state_trace) && ~isempty(r.state_trace)
 t=r.state_trace(1,1);
end
end
function [tol,kind]=tolerances(a,b,finite,path)
scale=ones(size(a));scale(finite)=max(1,abs(a(finite)));
tol=1e-12*ones(size(a));kind='absolute';leaf=leafName(path);
if endsWith(path,'.state_trace') && ismatrix(a) && size(a,2)==9
 kind='state';tol(:,2:end)=1e-12*scale(:,2:end);return
end
if endsWith(path,'.reference_trace') && ismatrix(a) && size(a,2)==12
 kind='reference';tol(:,8:9)=1e-12*scale(:,8:9);tol(:,10:12)=0;return
end
exactNames={'sequence','sequence_watermark','epoch','ordinal','winner_ordinal', ...
 'candidate_count','safe_candidate_count','cost_sample_count','pending_count','reference_sequence'};
if any(strcmp(leaf,exactNames)) || endsWith(leaf,'_count') || endsWith(leaf,'_ordinal') || ...
 endsWith(leaf,'_sequence') || contains(leaf,'_flag')
 kind='exact';tol(:)=0;return
end
if contains(path,'.final_controller.') || contains(path,'.final_rx.') || isCore(path)
 return
end
bodyNames={'ne','ur_mps','vr_mps','n_measured_radps','measurement_ne','ground_u_mps', ...
 'ground_v_mps','target_ne','end_ne_m','winner_end_ne_m','origin_ne_m', ...
 'origin_public_ur_mps','min_predicted_clearance_m','minimum_clearance_m','minimum_scored_clearance_m'};
if contains(path,'.costcomponents.') || any(strcmp(leaf,{'score','winner_score'})) || any(strcmp(leaf,bodyNames))
 kind='scaled';tol=1e-12*scale;
end
end
function name=leafName(path)
parts=strsplit(path,'.');name=regexprep(parts{end},'[\(\{].*$','');
end
function yes=isCore(path)
yes=contains(path,'.reference_core.') || contains(path,'.core.');
end
function linear=firstNumericIndex(mask,kind)
if any(strcmp(kind,{'state','reference'})) && ismatrix(mask)
 [row,column]=find(mask);[~,k]=min((row-1)*size(mask,2)+column);
 linear=sub2ind(size(mask),row(k),column(k));
else
 linear=find(mask,1,'first');
end
end
function yes=earlier(a,b)
% Known candidate discrepancies precede global metadata, then earliest time.
% Unknown time sorts after a recorded time within the same candidate.
oa=priorityOrdinal(a.ordinal);ob=priorityOrdinal(b.ordinal);
if oa~=ob,yes=oa<ob;return;end
ta=priorityTime(a.time);tb=priorityTime(b.time);yes=ta<tb;
end
function out=priorityOrdinal(value)
if finiteScalar(value),out=value;else,out=Inf;end
end
function out=priorityTime(value)
if finiteScalar(value),out=value;else,out=Inf;end
end
