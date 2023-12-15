A1 =ones(N);
A1 = A1.*-(eye(height(A1))-1);
A2 = [0 1 0 1; 1 0 1 1; 0 1 0 1; 1 1 1 0];
A3 = [0 1 0 1; 1 0 1 0; 0 1 0 1; 1 0 1 0];
A4 = [0 0 1 0; 0 0 1 0; 1 1 0 1; 0 0 1 0];
A5 = [0 0 0 1; 1 0 0 0; 0 1 0 0; 0 1 1 0];
A6 = [0 0 0 0; 1 0 1 0; 0 0 0 1; 0 0 0 0];
A7 = [0 0 0 0; 1 0 1 0; 0 0 0 0; 0 0 1 0];

G1 = graph(A1); G2 = graph(A2); G3 = graph(A3); G4 = graph(A4);
G5 = digraph(A5); G6 = digraph(A6); G7 = digraph(A7);

% Compute Laplacian matrices
L1 = diag(sum(A1)) - A1;
L2 = diag(sum(A2)) - A2;
L3 = diag(sum(A3)) - A3;
L4 = diag(sum(A4)) - A4;
L5 = diag(sum(A5, 2)) - A5;
L6 = diag(sum(A6, 2)) - A6;
L7 = diag(sum(A7, 2)) - A7;
%%

% Compute eigenvalues of Laplacian matrices
eigvals_L1 = eig(L1);
eigvals_L2 = eig(L2);
eigvals_L3 = eig(L3);
eigvals_L4 = eig(L4);
eigvals_L5 = eig(L5);
eigvals_L6 = eig(L6);
eigvals_L7 = eig(L7);


% Sort eigenvalues in ascending order
eigvals_L1 = sort(eigvals_L1)
eigvals_L2 = sort(eigvals_L2)
eigvals_L3 = sort(eigvals_L3)
eigvals_L4 = sort(eigvals_L4)
eigvals_L5 = sort(eigvals_L5)
eigvals_L6 = sort(eigvals_L6)
eigvals_L7 = sort(eigvals_L7)

% Algebraic connectivity (second smallest eigenvalue)
algebraic_connectivity_1 = eigvals_L1(2)
algebraic_connectivity_2 = eigvals_L2(2)
algebraic_connectivity_3 = eigvals_L3(2)
algebraic_connectivity_4 = eigvals_L4(2)
algebraic_connectivity_5 = eigvals_L5(2)
algebraic_connectivity_6 = eigvals_L6(2)
algebraic_connectivity_7 = eigvals_L7(2)
%%

figure

sgtitle('Different graphs of communication')

subplot(2, 2, 1)
plot(G1)
title('G1')

subplot(2, 2, 2)
plot(G2)
title('G2')

subplot(2, 2, 3)
plot(G3)
title('G3')

subplot(2, 2, 4)
plot(G4)
title('G4')

%%

figure

sgtitle('Different graphs of G5, G6 and G7')

subplot(3, 1, 1)
plot(G5)
title('G5')

subplot(3, 1, 2)
plot(G6)
title('G6')

subplot(3, 1, 3)
plot(G7)
title('G7')

