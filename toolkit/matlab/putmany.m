function getmany(component,pv,values);

srlist = 1:24;
numlist = 1:7;
npoints = 0;

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
lcaPut(pvs.', values.');
