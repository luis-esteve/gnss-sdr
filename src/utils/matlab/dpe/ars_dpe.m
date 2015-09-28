disp(['The number of channels is ',num2str(trk_channels)])
s = size(correlators);
disp(['The size of correlators matrix is ',num2str(s)])
disp(['Radius[1] = ', num2str(radius(1))])
disp(['Radius[2] = ', num2str(radius(2))])

modcorrelators = abs(correlators);

% for i = 1:s(1)
%     for j = 1:s(2)
%         disp(['correlators[',num2str(i),',', num2str(j), '] =  ', num2str(real(correlators(i,j))), '+j', num2str(imag(correlators(i,j))) ])
%     end
% end

disp(['n_iter = ', num2str(n_iter)])
disp(['cf = ', num2str(cf)])

disp(['x = ', num2str(pos_0(1)), ' m'])
disp(['y = ', num2str(pos_0(2)), ' m'])
disp(['z = ', num2str(pos_0(2)), ' m'])
disp(['dt = ', num2str(dt), ' s'])

space_chips = [-2, -1, -0.5, -0.25, 0, 0.25, 0.5, 1, 2];
subplot(321), plot(space_chips,modcorrelators(:,1),'-.s');
subplot(322), plot(space_chips,modcorrelators(:,2),'-.s'); 
subplot(323), plot(space_chips,modcorrelators(:,3),'-.s'); 
subplot(324), plot(space_chips,modcorrelators(:,4),'-.s'); 
subplot(325), plot(space_chips,modcorrelators(:,5),'-.s'); 
%grid on;
%title('Correladores');
%title(sprintf('Channel: %u \t PRN = %u \t Number of codes = %u',mat_channel,mat_prn,mat_count));
%hold on;  
%plot(epl_index, epl_corr,'rs');
%hold off;
%xlabel('samples');
%ylabel('Correlators');

