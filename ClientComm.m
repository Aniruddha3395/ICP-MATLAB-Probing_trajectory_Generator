classdef ClientComm
    properties
        ip_address;
        port;
        tcp;
    end
    
    methods
        function comm = ClientComm(ip_address,port)
            comm.ip_address = ip_address;
            comm.port = port;
            comm.tcp = tcpip(comm.ip_address,comm.port,'NetworkRole','Client');
            fopen(comm.tcp);
            fprintf(comm.tcp,'Establish Communication');
            data = fscanf(comm.tcp,'%s');
            if (strcmp(data,'Okay')==1)
                disp('communication_established to the robot!')
            end
        end
        
        function send_data(comm,data)
            fprintf('Sending data to the server at %s\n',comm.ip_address);
            fprintf(comm.tcp,data);
        end
        
        function data = receive_data(comm)
            fprintf('Receiving data from the server at %s\n',comm.ip_address);
            data = fscanf(comm.tcp,'%s');
        end
        
        function close_comm(comm)
            delete(comm.tcp);
        end
        
    end 
    
end