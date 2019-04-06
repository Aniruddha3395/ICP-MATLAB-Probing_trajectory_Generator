function ExecuteMotion(~,~)

global param
a = ClientComm('192.168.10.114',param.port);
a.send_data('execute');
a.receive_data();
a.close_comm();

end