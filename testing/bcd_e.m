bpms=[1 2 3 4];
power=16:-.5:6;
sml(['pow ' num2str(power(1))]);
pause(15)
for p=1:length(power)
    sml(['pow ' num2str(power(p))]);
    pause(.1)
    for i=bpms
        x(p,i)=lcaGet(['TS-DI-EBPM-0' num2str(i) ':SA:X'])*1000;
        y(p,i)=lcaGet(['TS-DI-EBPM-0' num2str(i) ':SA:Y'])*1000;
        c(p,i)=lcaGet(['TS-DI-EBPM-0' num2str(i) ':SA:CURRENT']);
    end
end
for i=bpms
    figure(i)
    plot(c(:,i),(x(:,i)-x(1,i)),'b',c(:,i),(y(:,i)-y(1,i)),'r')%,[1 10 10 60 60 300],[100 100 50 50 1 1],'g--')
%    axis([150 300 -10 10])
    title(['TS-DI-EBPM-0' num2str(i)])
    xlabel('beam current [mA]')
    ylabel('position deviation [um]')
    drawnow
end
