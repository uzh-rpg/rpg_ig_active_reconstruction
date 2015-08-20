% subtracts the first time from all following times in the vector
function time_vec = normalizeTime(time_vec_before)
    start_time = time_vec_before(1);
    for i=1:length(time_vec_before)
        time_vec(i) = (time_vec_before(i)-start_time)/60;
    end
end