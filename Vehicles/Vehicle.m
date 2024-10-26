classdef Vehicle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m (1, 1) double
        L (1, 1) double
        I (1, 1) double
        Name (1,1) string
    end
    
    methods
        function obj = Vehicle(m, L, I, Name)
            %VEHICLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.m = m;
            obj.L = L;
            obj.I = I;
            obj.Name = Name;
        end

        function save(obj)
            vehicle = obj;
            path = string("Vehicles\" + obj.Name + ".mat");
            save(path, "vehicle");
        end
    end
end

