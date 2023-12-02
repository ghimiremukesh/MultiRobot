%% create_potential_based_controller
% Returns controller for formation control
%% Detailed Description
%%
%  potential_fun - potential function describing the formation shape
%% Implementation
function [ created_potential_controller ] = create_potential_controller()
    global x y z
    syms x y z
    R_0 = 2.4 * sqrt(2); a=2.0; b=0.3; c=5; d=pi/2;
    gamma = (sqrt(x.^2 + y.^2) - (a + b*sin(c*atan2(y, x) + d)));
    beta_0 = R_0 - sqrt(x.^2 + y.^2);
    nav_sym = (gamma.^2)./(gamma.^2 + beta_0);
    dndx_sym = vpa(diff(nav_sym, 'x'));
    dndy_sym = vpa(diff(nav_sym, 'y'));
    psi = [0;0;gamma];
    curl_sym = vpa(curl(psi, [x, y, z]));
    created_potential_controller = @position_kni_clf;
    function [ dx ] = position_kni_clf(states)
        states = states';
        x = states(:, 1)';
        y = states(:, 2)';
        z=0;
        dndx = double(subs(dndx_sym));
        dndy = double(subs(dndy_sym));
        curl = double(subs(curl_sym));
        grad = [dndx' dndy'];
        curl = curl(1:2, :)';
        dx = -grad - curl;
        dx = dx';
    end
end