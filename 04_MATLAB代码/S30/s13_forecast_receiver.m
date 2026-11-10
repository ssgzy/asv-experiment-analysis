function [reference,rx,events,cacheUse] = s13_forecast_receiver(t,incoming,rx,settings,supervisor)
%S06_FORECAST_RECEIVER Isolated forecast receiver with validated constant-channel cache.
% Native validation, state transitions and shaping are retained verbatim.
% Incoming is a cell array so malformed heterogeneous packets remain data.
% Sequence is global across planners. Only active references advance the
% sequence/generation watermark; future-start packets cannot reserve it.
% Controller/plant/estimator state is intentionally absent from this API.
if nargin<5 || isempty(supervisor),supervisor=struct('trigger',false,'clear_authorized',false,'reason','');end
validateattributes(t,{'double'},{'scalar','real','finite','nonnegative'});
events=emptyEvents();
if nargout>3,cacheUse=struct('ur_mode','none','psi_mode','none');end
if isempty(rx)
    rx=struct('active',[],'pending',{{}},'sequence_watermark',-1,'generation_watermark',-Inf, ...
        'shaped_heading_rad',settings.initial_heading_rad,'last_time_s',t, ...
        'supervisor_latched',false,'supervisor_reason','','latch_started_at_s',NaN);
end
assert(t>=rx.last_time_s,'p0ref:NonmonotonicClock','Receiver clock moved backwards.');
dt=t-rx.last_time_s;
rx.last_time_s=t;
if isempty(incoming),incoming={};elseif ~iscell(incoming)
    if isstruct(incoming),incoming=num2cell(incoming);else,incoming={incoming};end
end

% Pending packets are reconsidered without changing their receive timestamps.
pending=rx.pending;rx.pending={};
for k=1:numel(pending)
    p=pending{k};
    if t>=p.valid_until
        events=addEvent(events,t,'rejected',p,'expired');
    elseif p.sequence<=rx.sequence_watermark
        events=addEvent(events,t,'rejected',p,'stale_sequence');
    elseif p.generated_from_time<rx.generation_watermark
        events=addEvent(events,t,'rejected',p,'stale_generation');
    elseif p.t_ref(1)<=t
        [rx,events]=activate(rx,p,t,events);
    else
        rx.pending{end+1}=p; %#ok<AGROW>
    end
end

for k=1:numel(incoming)
    raw=incoming{k};[p,reason]=validatePacket(raw,t,settings);
    if ~isempty(reason),events=addEvent(events,t,'rejected',raw,reason);continue;end
    if p.sequence<=rx.sequence_watermark
        events=addEvent(events,t,'rejected',p,'stale_sequence');continue;
    end
    if p.generated_from_time<rx.generation_watermark
        events=addEvent(events,t,'rejected',p,'stale_generation');continue;
    end
    duplicate=false;
    for j=1:numel(rx.pending),duplicate=duplicate || rx.pending{j}.sequence==p.sequence;end
    if duplicate,events=addEvent(events,t,'rejected',p,'duplicate_pending_sequence');continue;end
    events=addEvent(events,t,'accepted',p,'validated_causal_packet');
    if p.t_ref(1)>t
        if numel(rx.pending)>=settings.pending_capacity
            events=addEvent(events,t,'rejected',p,'pending_capacity');
        else
            rx.pending{end+1}=p; %#ok<AGROW>
            events=addEvent(events,t,'queued',p,'future_valid_start');
        end
    else
        [rx,events]=activate(rx,p,t,events);
    end
end

if ~isempty(rx.active) && (t>=rx.active.valid_until || t>rx.active.t_ref(end))
    events=addEvent(events,t,'expired',rx.active,'half_open_validity_ended');rx.active=[];
end
fresh=~isempty(rx.active) && t>=rx.active.t_ref(1) && t<rx.active.valid_until && t<=rx.active.t_ref(end);
if logical(supervisor.trigger)
    if ~rx.supervisor_latched
        rx.latch_started_at_s=t;events=addEvent(events,t,'supervisor_latched',[],'common_supervisor_trigger');
    end
    rx.supervisor_latched=true;rx.supervisor_reason=char(supervisor.reason);
elseif rx.supervisor_latched && logical(supervisor.clear_authorized) && fresh
    rx.supervisor_latched=false;rx.supervisor_reason='';
    events=addEvent(events,t,'supervisor_cleared',rx.active,'authorized_clear_with_valid_reference');
end

reference=struct('speed_mps',0,'heading_rad',rx.shaped_heading_rad,'raw_heading_rad',rx.shaped_heading_rad, ...
    'raw_speed_mps',0,'sequence',-1, ...
    'fresh',logical(fresh),'fallback',~logical(fresh));
if fresh
    p=rx.active;
    reference.sequence=p.sequence;
    % Per-channel cache is internal to successfully validated packets only.
    % A receiver state from the original backend has no cache and uses interp1.
    if all(isfield(p,{'s06_const_ur','s06_ur_value'})) && p.s06_const_ur
        reference.raw_speed_mps=p.s06_ur_value;
        if nargout>3,cacheUse.ur_mode='constant_cache';end
    else
        reference.raw_speed_mps=interp1(p.t_ref,p.ur_ref,t,'linear');
        if nargout>3,cacheUse.ur_mode='interp1';end
    end
    if all(isfield(p,{'s06_const_psi','s06_psi_value'})) && p.s06_const_psi
        reference.raw_heading_rad=p.s06_psi_value;
        if nargout>3,cacheUse.psi_mode='constant_cache';end
    else
        reference.raw_heading_rad=interp1(p.t_ref,p.psi_unwrapped,t,'linear');
        if nargout>3,cacheUse.psi_mode='interp1';end
    end
end
% Supervisor and fallback hold the LAST shaped heading. Incoming candidates
% can update the stored reference but cannot overwrite the override command.
if fresh && ~rx.supervisor_latched
    delta=atan2(sin(reference.raw_heading_rad-rx.shaped_heading_rad),cos(reference.raw_heading_rad-rx.shaped_heading_rad));
    bound=settings.heading_rate_limit_radps*dt;
    rx.shaped_heading_rad=rx.shaped_heading_rad+max(-bound,min(bound,delta));
    reference.speed_mps=reference.raw_speed_mps;
end
reference.heading_rad=rx.shaped_heading_rad;
end

function [rx,events]=activate(rx,p,t,events)
if p.sequence<=rx.sequence_watermark
    events=addEvent(events,t,'rejected',p,'stale_sequence');return;
end
if p.generated_from_time<rx.generation_watermark
    events=addEvent(events,t,'rejected',p,'stale_generation');return;
end
rx.active=p;rx.sequence_watermark=p.sequence;rx.generation_watermark=p.generated_from_time;
events=addEvent(events,t,'activated',p,'valid_reference_replacement');
end

function [p,reason]=validatePacket(raw,t,s)
p=[];reason='';
required={'reference_id','planner_id','sequence','generated_from_time','ready_time','valid_until', ...
    't_ref','psi_ref','ur_ref','status','failure_reason'};
if ~isstruct(raw) || ~isscalar(raw),reason='not_scalar_packet';return;end
if ~all(isfield(raw,required)),reason='missing_field';return;end
for name={'reference_id','planner_id','status','failure_reason'}
    value=raw.(name{1});
    if ~(ischar(value) && (isrow(value) || isempty(value))) && ~(isstring(value) && isscalar(value))
        reason='invalid_text_field';return;
    end
end
if isempty(char(raw.reference_id)) || isempty(char(raw.planner_id)),reason='empty_identity';return;end
if ~strcmp(char(raw.status),'ok'),reason='planner_failure';return;end
if ~isempty(char(raw.failure_reason)),reason='ok_packet_has_failure_reason';return;end
for name={'sequence','generated_from_time','ready_time','valid_until'}
    value=raw.(name{1});
    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value)
        reason='invalid_scalar_field';return;
    end
end
if raw.sequence<0 || raw.sequence~=floor(raw.sequence) || raw.sequence>flintmax
    reason='invalid_sequence';return;
end
if raw.generated_from_time<0 || raw.ready_time<raw.generated_from_time || raw.valid_until<0
    reason='invalid_time_relation';return;
end
if raw.ready_time>t+1e-10,reason='not_ready';return;end
if t>=raw.valid_until,reason='expired';return;end
if raw.ready_time-raw.generated_from_time>s.compute_budget_s+1e-10
    reason='compute_budget';return;
end
if t-raw.generated_from_time>s.max_arrival_age_s+1e-10
    reason='arrival_too_old';return;
end
for name={'t_ref','psi_ref','ur_ref'}
    a=raw.(name{1});
    if ~isnumeric(a) || ~isvector(a) || ~isreal(a) || isempty(a)
        reason='invalid_sample_vector';return;
    end
    if ~all(isfinite(a)),reason='nonfinite_value';return;end
end
times=raw.t_ref(:);psi=raw.psi_ref(:);ur=raw.ur_ref(:);
if numel(times)~=numel(psi) || numel(times)~=numel(ur) || numel(times)<2
    reason='sample_length_mismatch';return;
end
if any(diff(times)<=0),reason='nonmonotonic_time';return;end
if any(abs(diff(times)-s.node_dt_s)>1e-9),reason='wrong_node_spacing';return;end
if abs(times(end)-times(1)-s.reference_horizon_s)>1e-9
    reason='wrong_reference_horizon';return;
end
if times(1)<raw.generated_from_time || raw.valid_until<=times(1) || raw.valid_until>times(end)+1e-10
    reason='invalid_validity_window';return;
end
if any(ur<0 | ur>s.speed_max_mps),reason='out_of_range_speed';return;end
if any(psi < -pi-1e-12 | psi>pi+1e-12),reason='out_of_range_heading';return;end
p=struct('reference_id',char(raw.reference_id),'planner_id',char(raw.planner_id), ...
    'sequence',double(raw.sequence),'generated_from_time',double(raw.generated_from_time), ...
    'ready_time',double(raw.ready_time),'valid_until',double(raw.valid_until),'t_ref',times, ...
    'psi_ref',psi,'psi_unwrapped',unwrap(psi),'ur_ref',ur,'status','ok','failure_reason','');
% Cache only after every native validation has succeeded and psi is unwrapped.
% Exact equality is required; the speed and heading channels are independent.
p.s06_const_ur=all(isfinite(p.ur_ref)) && all(p.ur_ref==p.ur_ref(1));
p.s06_ur_value=p.ur_ref(1);
p.s06_const_psi=all(isfinite(p.psi_unwrapped)) && all(p.psi_unwrapped==p.psi_unwrapped(1));
p.s06_psi_value=p.psi_unwrapped(1);
end

function events=emptyEvents()
events=struct('t_s',{},'event',{},'reference_id',{},'planner_id',{},'sequence',{}, ...
    'generated_from_time',{},'ready_time',{},'valid_until',{},'reason',{});
end

function events=addEvent(events,t,event,packet,reason)
row=struct('t_s',t,'event',event,'reference_id','','planner_id','','sequence',NaN, ...
    'generated_from_time',NaN,'ready_time',NaN,'valid_until',NaN,'reason',reason);
if isstruct(packet) && isscalar(packet)
    for name={'reference_id','planner_id'}
        key=name{1};if isfield(packet,key) && (ischar(packet.(key)) || (isstring(packet.(key)) && isscalar(packet.(key))))
            row.(key)=char(packet.(key));
        end
    end
    for name={'sequence','generated_from_time','ready_time','valid_until'}
        key=name{1};if isfield(packet,key) && isnumeric(packet.(key)) && isscalar(packet.(key)) && isreal(packet.(key))
            row.(key)=double(packet.(key));
        end
    end
end
events(end+1)=row;
end
