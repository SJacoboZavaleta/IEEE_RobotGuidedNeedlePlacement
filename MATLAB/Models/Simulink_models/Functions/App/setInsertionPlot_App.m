function [u,v,c_ext,phi] = setInsertionPlot_App(p_c,holderModel)
%% Defining initial data for plotting breast holder profile function
% INPUTS
% p_c: Biopsy target in breast holder frame origin {C}
% holderModel .mat model of desired holder
% OUTPUTS
% u, v: Auxiliar unit vectors for geometrical construction
% c_ext: Vector position for upper limit 
% phi: Half angle of the conical opening  
% --------------------------------------------------------------
%% Extracting some data
R_holderUpper = holderModel.R_upper;
R_holderLower = holderModel.R_lower;
H_holder = holderModel.H_holder;
phi = holderModel.phi;
T_holder = holderModel.T_holder;

%% XoYo Projection
rxy = sqrt(p_c(1)^2 + p_c(2)^2);
u = [p_c(1) p_c(2) 0]' / rxy;% horizontal vector in radial plane
v = [-p_c(2) p_c(1) 0]'/rxy;% perpendicular to u and going out from plane
%n : needle insertion direction insie the plane
%w : unit vector along the generatrix side

%% On the radial plane. RadiusP is the distance from axis Zc to its intersecion with breast
% holder device outside in the XoYo projection of biopsy target along direction u
radiusP = (R_holderUpper-R_holderLower)*(H_holder-p_c(3))/H_holder + R_holderLower;%r
d1 = radiusP - rxy;

%% Point C and its projection on the breast holder outside
c = p_c + d1*u;
c_ext = c + u*T_holder/sin(phi);
end