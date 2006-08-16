function raw=getraw(name)
%getraw  Retrieves raw ADC waveform from named Libera
%
% raw = getraw(name) returns a 256 by 4 element array with one column for
%    each button.
raw(1:1024,1)=lcaGet([name ':FT:RAWA']);
raw(1:1024,2)=lcaGet([name ':FT:RAWB']);
raw(1:1024,3)=lcaGet([name ':FT:RAWC']);
raw(1:1024,4)=lcaGet([name ':FT:RAWD']);
