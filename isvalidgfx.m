function tf = isvalidgfx(h)
    tf = ~isempty(h) && isgraphics(h) && isvalid(h);
end