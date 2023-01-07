function [a_c,n_c,dInsertionPoint] = setInsertionPoint_Sim(p_c,u,v,alpha,phi,c_ext)
%% Defining position and direction for insertion point through breast holder 
% INPUTS
% p_c: Biopsy target position in frame {c}
% u,v: Auxiliar vectors
% alpha: Angle between cone generatrix and n_c
% phi: Half angle of breast holder (truncated cone) 
% c_ext: Point outside of breast holder's surface. Horizontal proyection of
%       p_c
% OUTPUTS
% a_c: Insertion point in the inner surface of breast holder in frame {c}
% n_c: Needle insertion direction in frame {c} 
% dInsertionPoint: Needle insertion distance along n_c direction from a_c to p_c.
% --------------------------------------------------------------
%% Calculating new insertion direction
screwMatrix= VecToso3(-v);
Rb = MatrixExp3s(screwMatrix,alpha);
n_c = Rb*u;%Needle insertion directions

% Finding distance from p to new entry a_new
% Using law of sines formula
% From p to b : d1*sin(phi)/sin(betha);
dInsertionPoint = norm(c_ext-p_c)*sin(phi)/sin(pi-phi-alpha);
a_c = p_c + n_new*dInsertionPoint;%Insertion point in the inner surface of breast Holder

end