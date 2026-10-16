function rosmsgOut = Nums(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.num1 = uint64(slBusIn.num1);
    rosmsgOut.num2 = uint64(slBusIn.num2);
end
