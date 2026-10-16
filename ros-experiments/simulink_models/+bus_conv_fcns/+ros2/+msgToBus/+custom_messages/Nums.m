function slBusOut = Nums(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    if (nargin == 5) && varargin{3} % Cast 64-bit integers to double
        slBusOut.num1 = double(msgIn.num1);
    else
        slBusOut.num1 = uint64(msgIn.num1);
    end
    if (nargin == 5) && varargin{3} % Cast 64-bit integers to double
        slBusOut.num2 = double(msgIn.num2);
    else
        slBusOut.num2 = uint64(msgIn.num2);
    end
end
