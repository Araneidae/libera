function xy=getxy(name,len)
%getxy  Retrieve long XY waveforms
%
% xy = getxy(name, len) returns an array of complex values of dimension len
%    by 2 containing turn by turn data with one column for each position.
%
% The value for len should be a multiple of 16384 (or else the array returned
% will be truncated) and should be no longer than the programmed maximum
% capture length for Libera.
l=16384;
lcaPut([name ':TT:CAPLEN_S'],len);
lcaPut([name ':TT:LENGTH_S'],l);
lcaPut([name ':TT:READY'],0);
while lcaGet([name ':TT:READY'],1,'short')~=0; pause(.01); end
lcaPut([name ':TT:ARM'],1);
while lcaGet([name ':TT:READY'],1,'short')==0; pause(.01); end
for i=0:(floor(len/l)-1);
    lcaPut([name ':TT:OFFSET_S'],i*l);
    while lcaGet([name ':TT:OFFSET'])~=i*l; end
    xy((1+i*l):(l+i*l),1)=lcaGet([name ':TT:WFX']);
    xy((1+i*l):(l+i*l),2)=lcaGet([name ':TT:WFY']);
end
