classdef ControllerType < Simulink.IntEnumType
    enumeration

        No_Control(0),
        CasADi(1),
        CasADi_freetf(2),
    end
end
