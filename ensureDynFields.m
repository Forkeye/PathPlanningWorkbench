function dyn = ensureDynFields(dyn)
% Fill any missing fields in a struct array
    need = {'start','end','size','speed','pos','hPatch'};
    for i = 1:numel(dyn)
        if ~isfield(dyn,'start') && isfield(dyn,'pos')
            dyn(i).start = dyn(i).pos;
        end
        for f = 1:numel(need)
            fn = need{f};
            if ~isfield(dyn,fn) || isempty(dyn(i).(fn))
                switch fn
                    case 'start', dyn(i).start = [0 0];
                    case 'end',   dyn(i).end   = dyn(i).start;
                    case 'size',  dyn(i).size  = 6;
                    case 'speed', dyn(i).speed = 1;
                    case 'pos',   dyn(i).pos   = dyn(i).start;
                    case 'hPatch',dyn(i).hPatch= [];
                end
            end
        end
    end
end