module half_adder_tb();
    reg a,b;// t_a,t_b;
    wire sum,carry; //t_y;
    half_adder dut(sum,carry,a,b);
    initial begin
    #5 
    a=0;b=0; //#5 means after 5ns
    #10 
    a=0;b=1;
    #10 
    a=1;b=0;
    #10
    a=1;b=1;
    #10
    $stop;
    end
    endmodule