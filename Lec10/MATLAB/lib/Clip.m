function y = Clip(x, lowerValue, upperValue)
    y=min(max(x,lowerValue),upperValue);
end