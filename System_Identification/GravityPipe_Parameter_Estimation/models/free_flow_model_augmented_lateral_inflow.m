function [dx, y] = free_flow_model_augmented_lateral_inflow(t, x, u, p1, p2, p3, p4, p5,N_states , N_optimization_variables, N_aug_states, varargin)
% Continous time nlgreyest model for the diffusion wave gravity pipe with the tank in the end of the pipe. 
Augmented_states = zeros(1,N_states+1);
for i=1:N_states+1
    Augmented_states(i) = cell2mat(varargin(i));
end
dx = zeros(N_states+N_aug_states,1);
y = zeros(N_optimization_variables,1);

N1 = Augmented_states(1);
N2 = Augmented_states(2);
N3 = Augmented_states(3);
N4 = Augmented_states(4);
N5 = Augmented_states(5);
%% Augmented states
if N1 > 1
    dx(N_states + 1) = p1 * u(1) - p2 * x(N_states + 1) + p3*x(N_states + 2)-p4;
    for j = 2:N1-1
        dx(N_states + j) = p2*x(N_states+j-1) - (p2+p3)*x(N_states + j) + p3*x(N_states + j+1);
    end
    dx(N_states + N1) = p2*x(N_states+N1-1) - (p2+p3)*x(N_states + N1) + p3*x(1);
elseif N1 == 1
    dx(N_states + 1) = p1 * u(1) - p2 * x(N_states + 1) + p3*x(1)-p4;
end

%% State equations
% Sensor 1
if N1 == 0
    if N2 == 0
        dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(2)-p4;
    else
        dx(1) =  p1 * u(1) - p2 * x(1) + p3*x(N_states + N1+1)-p4;
    end
else
    if N2 == 0
        dx(1) =  p2*x(N_states+N1) - (p2+p3)*x(1) + p3*x(2);
    else
        dx(1) =  p2*x(N_states+N1) - (p2+p3)*x(1) + p3*x(N_states+N1+1);
    end
end 

if N2 > 1
    dx(N_states +N1+ 1) = p2*x(1) - (p2+p3)*x(N_states + N1+1) + p3*x(N_states+N1 + 2);
    for j = 2:N2-1
        dx(N_states+N1 + j) = p2*x(N_states+N1+j-1) - (p2+p3)*x(N_states+N1 + j) + p3*x(N_states+N1 + j+1);
    end
    dx(N_states + N1+N2) = p2*x(N_states+N1+N2-1) - (p2+p3)*x(N_states + N1+N2) + p3*x(2);
elseif N2 == 1
    dx(N_states + N1+ 1) = p2*x(1) - (p2+p3)*x(N_states + N1+1) + p3*x(2);
end
% Sensor 2
%dx(2) =  p2*x(N_states + N1+N2) - (p2+p3)*x(2) + p3*x(N_states + N1+N2+1); 
if N2 == 0
    if N3 == 0
        dx(2) =  p2 * x(1) - (p2+p3) * x(2) + p3*x(3)+ p1*u(3);
    else
        dx(2) =  p2 * x(1) - (p2+p3) * x(2) + p3*x(N_states + N1+N2+1)+ p1*u(3);
    end
else
    if N3 == 0
        dx(2) =  p2*x(N_states + N1+N2) - (p2+p3)*x(2) + p3*x(3)+p1*u(3); 
    else
        dx(2) =  p2*x(N_states + N1+N2) - (p2+p3)*x(2) + p3*x(N_states + N1+N2+1)+p1*u(3); 
    end
end 

if N3 > 1
    dx(N_states +N1+N2+ 1) = p2*x(2) - (p2+p3)*x(N_states + N1+N2+1) + p3*x(N_states+N1+N2 + 2);
    for j = 2:N3-1
        dx(N_states+N1+N2 + j) = p2*x(N_states+N1+N2+j-1) - (p2+p3)*x(N_states+N1+N2 + j) + p3*x(N_states+N1+N2 + j+1);
    end
    dx(N_states + N1+N2+N3) = p2*x(N_states+N1+N2+N3-1) - (p2+p3)*x(N_states + N1+N2+N3) + p3*x(3);
elseif N3 == 1
    dx(N_states + N1+N2+1) = p2*x(2) - (p2+p3)*x(N_states + N1+N2+1) + p3*x(3);
end
% Sensor 3
% if N4 == 0
%     dx(3) =  p2*x(N_states + N1+N2+N3) - (p2+p3)*x(3) + p3*x(4);
% else
%     dx(3) =  p2*x(N_states + N1+N2+N3) - (p2+p3)*x(3) + p3*x(N_states + N1+N2+N3+1); 
% end

if N3 == 0
    if N4 == 0
        %dx(3) =  p2 * x(2) - (p2+p3) * x(3) + p3*x(4) + p1*u(3);                    % Lateral inflow added
        dx(3) =  p2 * x(2) - (p2+p3) * x(3) + p3*x(4);
    else
        %dx(3) =  p2 * x(2) - (p2+p3) * x(3) + p3*x(N_states + N1+N2+N3+1)+ p1*u(3); % Lateral inflow added
        dx(3) =  p2 * x(2) - (p2+p3) * x(3) + p3*x(N_states + N1+N2+N3+1);
    end
else
    if N4 == 0
        dx(3) =  p2*x(N_states + N1+N2+N3) - (p2+p3)*x(3) + p3*x(4);
    else
        dx(3) =  p2*x(N_states + N1+N2+N3) - (p2+p3)*x(3) + p3*x(N_states + N1+N2+N3+1);
    end
end 

if N4 > 1
    dx(N_states +N1+N2+N3+ 1) = p2*x(3) - (p2+p3)*x(N_states + N1+N2+N3+1) + p3*x(N_states+N1+N2+N3 + 2);
    for j = 2:N4-1
        dx(N_states+N1+N2+N3 + j) = p2*x(N_states+N1+N2+N3+j-1) - (p2+p3)*x(N_states+N1+N2+N3 + j) + p3*x(N_states+N1+N2+N3 + j+1);
    end
    dx(N_states + N1+N2+N3+N4) = p2*x(N_states+N1+N2+N3+N4-1) - (p2+p3)*x(N_states + N1+N2+N3+N4) + p3*x(4);
elseif N4 == 1
    dx(N_states + N1+N2+N3+1) = p2*x(3) - (p2+p3)*x(N_states + N1+N2+N3+1) + p3*x(4);
end
% Sensor 4
% if N4 == 0
%     dx(4) = p2 * x(3) - p3*x(4)+ p4 -  p5*(x(4)-x(5)+tank_offset);
% else
%     dx(4) = p2 * x(N_states+N1+N2+N3+N4) - p3*x(4) + p4 -  p5*(x(4)-x(5)+tank_offset);
% end
if N4 == 0
    if N5 == 0
        dx(N_states) = p2 * x(N_states-1) - p3*x(N_states) + p4 -  p5*(x(N_states));
    else
        dx(4) =  p2*x(3) - (p2+p3)*x(4) + p3*x(N_states + N1+N2+N3+N4+1);
    end
else
    if N5 == 0
        dx(N_states) = p2 * x(N_states+N1+N2+N3+N4) - p3*x(N_states) + p4 -  p5*(x(N_states));
    else
        dx(4) =  p2*x(N_states + N1+N2+N3+N4) - (p2+p3)*x(4) + p3*x(N_states + N1+N2+N3+N4+1); 
    end
end 

if N5 > 1
    dx(N_states +N1+N2+N3+N4+1) = p2*x(4) - (p2+p3)*x(N_states + N1+N2+N3+N4+1) + p3*x(N_states+N1+N2+N3+N4+2);
    for j = 2:N5-1
        dx(N_states+N1+N2+N3+N4+j) = p2*x(N_states+N1+N2+N3+N4+j-1) - (p2+p3)*x(N_states+N1+N2+N3+N4 + j) + p3*x(N_states+N1+N2+N3+N4+ j+1);
    end
    dx(N_states + N1+N2+N3+N4+N5) = p2*x(N_states+N1+N2+N3+N4+N5-1) - (p2+p3)*x(N_states + N1+N2+N3+N4+N5) + p4 -  p5*(x(N_states + N1+N2+N3+N4+N5));
elseif N5 == 1
    dx(N_states + N1+N2+N3+N4+1) = p2*x(4) - (p2+p3)*x(N_states + N1+N2+N3+N4+1)+ p4 -  p5*(x(N_states + N1+N2+N3+N4+1));
end


%% Output equation
y(1:N_states) = x(1:N_states);
if N5 > 0
    y(N_optimization_variables) = p5/p1*(x(N_states + N1+N2+N3+N4+N5));
else
    y(N_optimization_variables) = p5/p1*(x(N_states));
end

end