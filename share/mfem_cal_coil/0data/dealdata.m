function x=dealdata(x)
for i=1:length(x)
    if x(i)>2^31-1
        x(i)=x(i)-2^32;
    end
    x(i)=x(i)*10^3*2.5/2^23/256;
end