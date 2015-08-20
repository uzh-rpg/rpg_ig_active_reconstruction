% integrates all values
function out = integrate(in)
    sum = 0;
    for i=1:length(in)
        sum = sum + in(i);
        out(i) = sum;
    end
end