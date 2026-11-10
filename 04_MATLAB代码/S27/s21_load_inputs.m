function [c,profileCfg,reference,goldenSettings,provenance]=s21_load_inputs(base)
% Read typed sealed gold only. No model, RNG, controller or initialization.
stages=fileparts(base);s10=fullfile(stages,'S10');s04=fullfile(stages,'S04');
paths={fullfile(s10,'run_001','frozen_cases.mat'), ...
 fullfile(s10,'run_001','A_D1','original.mat'), ...
 fullfile(s04,'run_001','D0','native_window.mat')};
expected={'10e06da7024a27c7825f7b6e6b41f7edf6c99626f3caefc8a149bf7221701a1a', ...
 '21433dc6acc8ea65e6eb6bede603ade4562b1039a5db30dc8a23d148fb05ad6a',''};
sources=struct('path',{},'sha256',{},'fields',{});
for k=1:3
 h=sha(paths{k});if ~isempty(expected{k}),assert(strcmp(h,expected{k}),'s21:GoldSourceChanged');end
 sources(k)=struct('path',paths{k},'sha256',h,'fields','');
end
f=load(paths{1},'cases','profileCfg');g=load(paths{2},'currentOriginal');s=load(paths{3},'cfg','settings');
assert(iscell(f.cases) && isequal(size(f.cases),[1,4]),'s21:GoldCasesShape');
c=f.cases{1};profileCfg=f.profileCfg;reference=g.currentOriginal;goldenSettings=s.settings;
assert(strcmp(c.id,'A_D1') && isequaln(c.ctx.cfg,profileCfg) && isequaln(s.cfg,profileCfg) && reference.candidate_count==72,'s21:GoldIdentity');
sources(1).fields='cases{1}=A_D1; profileCfg';sources(2).fields='currentOriginal';sources(3).fields='cfg; settings';
provenance=struct('schema','s21-typed-startup-input-v1','sources',sources,'mat_loads',3, ...
 'historical_common_loads',0,'noise_draws',0,'model_calls',0,'controller_calls',0, ...
 'rule','Typed MAT input and scientific golden; historical settings check only. Fresh S21 common is initialized exactly once.');
end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0,'s21:SourceRead');cl=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');m=java.security.MessageDigest.getInstance('SHA-256');m.update(b);
h=lower(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
