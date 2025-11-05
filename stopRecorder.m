function stopRecorder(rec)
    if ~rec.enabled, return; end
    if strcmp(rec.format,'MP4') && ~isempty(rec.writer)
        close(rec.writer);
    end
end
