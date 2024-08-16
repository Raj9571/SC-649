L = 2;
I = 21;
J = (I+1)/2;
dl = L/(I-1); 
T = 1.5;
num_steps = 1;
dt = T/num_steps;
a1 = 0.01;
a2 = 0.01;
a3 = 0.01;
a4 = 0.01;
a5 = 0.01;
a6 = 0.2;
v = 0.4;
w = 0;
state = zeros(I,I,90);
state(J,1,1) = 1;
iter = 0;
for t = 1:num_steps
    iter = iter + 1
    state_old = state;
    for i = 1:I
        for j = 1:I
            for k = 1:90
                b = P_mat((i-1)*dl,-(j-J)*dl,(k-1)*4*pi/180,v,w,dt,dl,a1,a2,a3,a4,a5,a6,I,J,state_old); 
                %b
                state(j,i,k) = sum(b,"all");
                %state
            end
        end
    end
    state = state/sum(state,"all");
end
B = sum(state,3)
contourf(B)

function L = p(x,y,phi,x_n,y_n,phi_n,v,w,dt,a1,a2,a3,a4,a5,a6) 
    if x == x_n && y == y_n
        u = 0;
    else
        u = 0.5*((x - x_n)*cos(phi) + (y - y_n)*sin(phi))/((y - y_n)*cos(phi) - (x - x_n)*sin(phi));
    end
    if isinf(u) || u > 1e9 || isnan(u)
        d = norm([x-x_n,y-y_n],2);
        p1 = normpdf(0,v-(d/dt),(a1*abs(v)+a2*abs(w))^0.5);
        p2 = normpdf(0,w,(a3*abs(v)+a4*abs(w))^0.5);
        p3 = normpdf(0,((phi_n-phi)/dt),(a5*abs(v)+a6*abs(w))^0.5);
        P = p1*p2*p3;
    else
        x_a = 0.5*(x + x_n) + u*(y - y_n);
        y_a = 0.5*(y + y_n) + u*(x_n - x);
        r_a = norm([x-x_a,y-y_a],2);
        dphi = atan2(y_n-y_a,x_n-x_a) - atan2(y-y_a,x-x_a);
        V = dphi*r_a/dt;
        W = dphi/dt;
        R = ((phi_n-phi)/dt)-W;
        p1 = normpdf(0,v-V,(a1*abs(v)+a2*abs(w))^0.5);
        p2 = normpdf(0,w-W,(a3*abs(v)+a4*abs(w))^0.5);
        p3 = normpdf(0,R,(a5*abs(v)+a6*abs(w))^0.5);
        P = p1*p2*p3;
    end
    L=P;
end

function M = P_mat(x_n,y_n,phi_n,v,w,dt,dl,a1,a2,a3,a4,a5,a6,I,J,state_old)
    L = zeros(I,I,90);
    for i = 1:I
        for j = 1:I
            for k = 1:90
                if state_old(j,i,k) > 0.001 
                    L(j,i,k) = p((i-1)*dl,-(j-J)*dl,(k-1)*4*pi/180,x_n,y_n,phi_n,v,w,dt,a1,a2,a3,a4,a5,a6)*state_old(j,i,k);
                else
                    L(j,i,k) = 0;
                end
            end
        end
    end
    M = L;
end