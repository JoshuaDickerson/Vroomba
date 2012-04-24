classdef mydate

    properties
        minute = 0;
        hour;
        day;
        month;
        year;
    end
    
    methods
        function obj=mydate(minute,hour,day,month,year)
            if(nargin >0)
                 obj.minute = minute;
                 obj.hour   = hour;
                 obj.day    = day;
                 obj.month  = month;
                 obj.year   = year;
            end
        end
        
        function obj = rollDay(obj, numdays)
            obj.day = obj.day + numdays;
        end
    end
    
end
