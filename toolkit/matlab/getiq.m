function iq=getiq(name,len)
%getiq  Retrieve long IQ waveforms
%
% iq = getiq(name, len) returns an array of complex values of dimension len
%    by 4 containing turn by turn data with one column for each button.
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
    iq((1+i*l):(l+i*l),1)=lcaGet([name ':TT:WFAI'])+j*lcaGet([name ':TT:WFAQ']);
    iq((1+i*l):(l+i*l),2)=lcaGet([name ':TT:WFBI'])+j*lcaGet([name ':TT:WFBQ']);
    iq((1+i*l):(l+i*l),3)=lcaGet([name ':TT:WFCI'])+j*lcaGet([name ':TT:WFCQ']);
    iq((1+i*l):(l+i*l),4)=lcaGet([name ':TT:WFDI'])+j*lcaGet([name ':TT:WFDQ']);
end
