classdef Queue < handle
    %% Queue
    % Queue(): define a que
    % empty(): return 1 if queue is empty
    % add(ele): add the element in the end of queue
    % pop(): remove and return the first element
    
    properties
        begin;
        rear;
        elements;
    end
    
    methods
        function que = Queue()
            que.begin = 1;
            que.rear = 1;
        end
        
        function bo = empty(que)
            if que.begin == que.rear
                bo = 1;
            else
                bo = 0;
            end
        end
        
        function add(que, ele)
            que.elements(que.rear) = ele;
            que.rear = que.rear + 1;
        end
        
        function ele = pop(que)
            ele = que.elements(que.begin);
            que.begin = que.begin + 1;
        end
    end
    
end

