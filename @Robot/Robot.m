classdef Robot < handle
    properties
        State = 'Running';
    end
    
    events
        StateChanged
    end
    
    methods
        function eStop(obj)
            fprintf('Emergency Stop Triggered! Reset, then Resume to continue. SENT FROM ROBOT CLASS\n');
            obj.State = 'Stopped';
            notify(obj, 'StateChanged');
        end
        
        function resetEStop(obj)
            if strcmp(obj.State, 'Stopped')
                fprintf('E-Stop has been reset. System is paused. SENT FROM ROBOT CLASS\n');
                obj.State = 'Paused';
                notify(obj, 'StateChanged');
            else
                fprintf('Cannot reset E-Stop while the system is in %s state.SENT FROM ROBOT CLASS\n', obj.State);
            end
        end
        
        function resumeOperations(obj)
            if strcmp(obj.State, 'Paused')
                fprintf('Operations resumed. SENT FROM ROBOT CLASS\n');
                obj.State = 'Running';
                notify(obj, 'StateChanged');
            else
                fprintf('Cannot resume operations while the system is in %s state.\n', obj.State);
            end
        end
    end
end
