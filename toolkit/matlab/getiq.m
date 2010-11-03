function iq = getiq(name, len)
%getiq  Retrieve long IQ waveforms
%
% iq = getiq(name, len) returns an array of complex values of dimension len
%    by 4 containing turn by turn data with one column for each button.
%
% The value for len should be no longer than the programmed maximum capture
% length for Libera.

% First figure out how long our window is.
l = lcaGet([name ':TT:MAXLENGTH']);
r = lcaGet([name ':TT:DOREFRESH_S']);

% Program in the desired capture length, set maximum readout window and
% finally trigger capture.
lcaPut([name ':TT:CAPLEN_S'], len);
lcaPut([name ':TT:LENGTH_S'], l);
lcaPut([name ':TT:DOREFRESH_S'], 0);
lcaPut([name ':TT:READY'], 0);

lcaPut([name ':TT:ARM'],1);
while lcaGet([name ':TT:READY'], 1, 'short') == 0; pause(.01); end

% See how many points we actually managed to read
captured = lcaGet([name ':TT:CAPTURED']);

iq = zeros(captured, 4);
read = 0;
while read < captured;
    % At each stage capture no more than the remaining data required, and no
    % more than the available capture window.
    window = min(l, captured - read);
    if window < l;
        lcaPut([name ':TT:LENGTH_S'], window);
        while lcaGet([name ':TT:LENGTH_S']) ~= window; end
    end

    lcaPut([name ':TT:OFFSET_S'], read);
    while lcaGet([name ':TT:OFFSET']) ~= read; end

    iq(read+1:read+window, 1) = GetIqWf(name, 'A', window);
    iq(read+1:read+window, 2) = GetIqWf(name, 'B', window);
    iq(read+1:read+window, 3) = GetIqWf(name, 'C', window);
    iq(read+1:read+window, 4) = GetIqWf(name, 'D', window);

    read = read + window;
end

lcaPut([name ':TT:DOREFRESH_S'], r);


function wf = GetIqWf(name, button, length)
wfi = lcaGet([name ':TT:WF' button 'I']);
wfq = lcaGet([name ':TT:WF' button 'Q']);
wf = wfi(1:length) + 1i * wfq(1:length);
