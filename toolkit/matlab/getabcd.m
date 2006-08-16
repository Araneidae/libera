function abcd=getabcd(name)
%getabcd  Retrieves ADC amplitude waveform from named Libera
%
% abcd = getabcd(name) returns a 256 by 4 element array with one row for
%    each button.
abcd(:,1)=lcaGet([name ':FT:WFA']);
abcd(:,2)=lcaGet([name ':FT:WFB']);
abcd(:,3)=lcaGet([name ':FT:WFC']);
abcd(:,4)=lcaGet([name ':FT:WFD']);
