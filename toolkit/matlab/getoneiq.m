function iq=getoneiq(name,which,len)
%getiq  Retrieve long IQ waveforms
%
% iq = getiq(name, len) returns an array of complex values of dimension len
%    containing turn by turn data for one button.
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
    iq((1+i*l):(l+i*l))=lcaGet([name ':TT:WF' which 'I'])+j*lcaGet([name ':TT:WF' which 'Q']);
end
