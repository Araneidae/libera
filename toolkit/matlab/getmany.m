function [values,timestamps]=getmany(component,pv,varargin);
if nargin>2
    srlist=varargin{1};
else
    srlist=1:24;
end
if nargin>3
    numlist=varargin{2};
else
    numlist=1:7;
end
if nargin>4
    npoints=varargin{3};
else
    npoints=0;
end

pvs={};
for m=srlist
    for n=numlist
        if iscell(pv)
            for p=1:length(pv)
                pvs{end+1} =sprintf('SR%02.0fC-DI-%s-%02.0f:%s',m,component,n,pv{p});
            end
        else
        pvs{end+1} =sprintf('SR%02.0fC-DI-%s-%02.0f:%s',m,component,n,pv);
        end
    end
end

[values, timestamps]=lcaGet(pvs.',npoints);
if iscell(pv)
    values=reshape(values,length(pv),length(srlist)*length(numlist)).';
end
timestamps = real(timestamps) + 1e-9*imag(timestamps);
