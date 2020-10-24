%% --- Visualize an inverted pendulum simulation --- %%

function visSim_cart_pole(DATA,ref)
t = DATA(:,1);
pos = DATA(:,2);
ang = DATA(:,3);

%% Initialize pendulum and cart orientation
H = .1; % height of cart
h = .6; % length of pendulum -> h = 2*l
R_init = [cos(ang(1)), -sin(ang(1)); % rotation matrix
          sin(ang(1)), cos(ang(1))];
cart_base = [0 0; 0 H];
cart_init = cart_base + [pos(1); 0]; % init cart position
pend_base = [0 0; 0 h];
pend_init = R_init*pend_base + [pos(1); H/2]; % init pendulum pos and ang

%% Size-configuration of the plot and figure 
pend_fig = figure;
hold on
x_lim = [min(pos(1),ref) - 0.3, max(pos(1),ref) + 0.3];
y_lim = [-0.3, 0.7];
ylim(y_lim)
xlim(x_lim)
daspect([1 1 1])
hp = abs(y_lim(1)) + abs(y_lim(2)); % plot height
wp = abs(x_lim(1)) + abs(x_lim(2)); % plot width
coord = get(pend_fig,'position');
% set the figure position: [bot_left_x  bot_left_y  width  height]
set(pend_fig, 'position', [coord(1)/2 coord(2) coord(4)*wp/hp coord(4)]);

%% Display initial pendulum-cart graphics
plot([ref ref],y_lim,'k--') % show reference position
C = plot(cart_init(1,:), cart_init(2,:), 'r','LineWidth',600*H);
drawnow
P = plot(pend_init(1,:), pend_init(2,:), 'b','LineWidth',25*h);
txt = title(['Time (s):  ',num2str(round(t(1),1))]);
txt.FontSize = coord(4)/30;
txt.HorizontalAlignment = 'left';
txt.Position = txt.Position + [-0.1 0 0];
drawnow
pause(1)

%% Visualize all the data
for j = 2:length(ang)
    hold on
    R = [cos(ang(j)), -sin(ang(j));
         sin(ang(j)), cos(ang(j))];
    pend = R*(pend_base) + [pos(j); H/2];
    cart = cart_base + [pos(j); 0];
    set(C,'XData', cart(1,:));
    set(P,'XData', pend(1,:));
    set(P,'YData', pend(2,:));
    set(txt, 'String', ['Time (s): ',num2str(round(t(j),1))]);
    drawnow;
    pause((-2.4+t(end))/length(t)); % Try to display in real-time
end
