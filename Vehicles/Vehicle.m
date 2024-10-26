classdef Vehicle
    %VEHICLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m (1, 1) double
        L (1, 1) double
        I (1, 1) double
        Len (1,1) double
        Name (1,1) string
    end
    
    methods
        function obj = Vehicle(m, L, Len, options)
            %VEHICLE Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                m
                L
                Len
                options.I = Vehicle.estimateMOI(m, Len, L)
                options.Name =  "vehicle"
            end
            obj.m = m;
            obj.L = L;
            obj.I = options.I;
            obj.Len = Len;
            obj.Name = options.Name;
        end

        function save(obj)
            vehicle = obj;
            path = string("Vehicles\" + obj.Name + ".mat");
            save(path, "vehicle");
        end
    end

    methods(Static)
        function I = estimateMOI(m, Len, L)
            I = (1/3) * m * (Len)^2 + m * (L)^2; 
        end
    end
end

