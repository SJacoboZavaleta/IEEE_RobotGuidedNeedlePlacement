%% Flexible Model of Breast
clc, clear
mainPath = startupGUI;
breastPath = strcat(mainPath,'\CAD_models\Breast\cupAbreast2.stl');

figure
trisurf(stlread(breastPath))
axis equal

%% Step 1: Setting model properties
% For breast glandular and fatty tissue
% Comparaciones ex vs in en Ramiao2016

% In vivo
%Young's modulus in Pa: E 0.1-4k Mira2018
% Eright = 0.3kPa, Eleft = 0.2kPa, Eskin=4kPa Mira2018
% ex vivo:3-60kPa
% in vivo:0.2-6kPa
% superficial layer of superficial fascia: 88.12um
% skin : 2mm
% Eskin_min=7.4 (Han2013); Eskin_max=58.4 (Hendricks2006)
% quasi‐incompressible (Poisson’s ratio = 0.49) for 
%       hyper‐elastic Neo‐Hookean solids

% From lorenzen2002
%breast adipose tissue, breast parenchyma, benign tumor 
%tissue, and malignant tumor tissue were 1.7 kPa, 2.5 kPa, 
%7.0 kPa, and 15.9 kPa, respectively
% From McKignt2002
% G: 3.3 kPa, 7.5 kPa, and 33 kPa: adipose, fibroglandular
% tumor

%From xydeas2005
% lesion sizes 0.5 to 4.5 cm, mean 2.1cm

%From chen2013
%invasive ductal carcinoma is about three times stiffer than
%the adipose tissue and 1.5 times stiffer than the glandular
%tissue.

E_breast = 1.7e3;%En Cheng2013
rho_breast = 1020;% Mass density kg/m^3 ICR110
nu_breast = 0.3; % Poisson's ratio 0.495

% For skin tissue
% From sinkus for FE: nu=0.48
%From ICR110
% skin: vol=2496.8 cm3; mass=2721.5 g.
% density = mass/vol
% female skin thickness: 1.775 mm
% From Barufaldi2016
% tambien de 1.2 y 1.5
E_skin = 4e3;%Mira2018
rho_skin = 2721.5/2496.8*1000;% about 1.1 g/cm3 or kg/m3
nu_skin = 0.4;%aprox

% For suspicious mass
E_tar = 1.42e3;%ductal carcinoma Cheng2013
%16.42 invasive ductal carcinoma srivastava2011
%3.5-4 carcinoma sinkus2000
rho_tar = 1160;% about 1.16-1.19 g/cm3 ohno1977
nu_tar = 0.48;%aprox

%% Step 2: Specifying locations of interface frames
origins = [0 0 0
           0 0 -56e-3]; % Base frame
%origins = [0 0 0]; % Base frame
%origins = [0 0 -56]; % Base frame
numFrames = size(origins,1);

%% Step 3: Create the Finite-Element Mesh: Breast
feModel = createpde('structural','modal-solid');
importGeometry(feModel,breastPath);
structuralProperties(feModel, ...
    'YoungsModulus',E_breast, ...
    'PoissonsRatio',nu_breast, ...
    'MassDensity',rho_breast);
repMesh = generateMesh(feModel, ...
    'GeometricOrder','linear',...
    'Hmax',8e-3, ...
    'Hmin',2e-3);

%% Step 4: Set up the Multipoint Constraints for the Interface Frames
figure
pdegplot(feModel,'FaceLabels','on','EdgeLabels','on','FaceAlpha',0.5)

%faceIDs = [5 4];   % model 3: No solution en 20min
%faceIDs = [3 2];   % model 4
%faceIDs = 3;       % model 4
IDs = [2,1];
type = ["Face","Face"];

figure
pdemesh(feModel,'FaceAlpha',0.5)
hold on
colors = ['rgb' repmat('k',1,numFrames-3)];
assert(numel(IDs) == numFrames);
for k = 1:numFrames
    nodeIdxs = findNodes(feModel.Mesh,'region',type(k),IDs(k));
    scatter3( ...
        feModel.Mesh.Nodes(1,nodeIdxs), ...
        feModel.Mesh.Nodes(2,nodeIdxs), ...
        feModel.Mesh.Nodes(3,nodeIdxs), ...
        'ok','MarkerFaceColor',colors(k))
    scatter3( ...
        origins(k,1), ...
        origins(k,2), ...
        origins(k,3), ...
        80,colors(k),'filled','s')
end
hold off

for k = 1:numFrames
    structuralBC(feModel, ...
        type(k),IDs(k), ...
        'Constraint','multipoint', ...
        'Reference',origins(k,:));
end

%% Step 5: Generate the Reduced-Order Model
clc
rom = reduce(feModel,'FrequencyRange',[0,1e5]);

%% Computing viscous parameters
breast.P = rom.ReferenceLocations';  % Interface frame locations (n x 3 matrix)
breast.K = rom.K;                    % Reduced stiffness matrix
breast.M = rom.M;                    % Reduced mass matrix

dampingRatio = 0.05;
breast.C = computeModalDampingMatrix(dampingRatio,rom.K,rom.M);

%%
%The boundary nodes in the reduced-order model must be specified 
% in the same order as the corresponding interface frames on the block.
% This order is given by the rows of the array origins. If the order of 
% the MPCs is different than the order specified by origins, permute the 
% rows and columns of the various matrices so that they match the original order.
frmPerm = zeros(numFrames,1);    % Frame permutation vector
dofPerm = 1:size(breast.K,1);    % DOF permutation vector

assert(size(breast.P,1) == numFrames);
for i = 1:numFrames
    for j = 1:numFrames
        if isequal(breast.P(j,:),origins(i,:))
            frmPerm(i) = j;
            dofPerm(6*(i-1)+(1:6)) = 6*(j-1)+(1:6);
            continue;
        end
    end
end

assert(numel(frmPerm) == numFrames);
assert(numel(dofPerm) == size(breast.K,1));

breast.P = breast.P(frmPerm,:);
breast.K = breast.K(dofPerm,:);
breast.K = breast.K(:,dofPerm);
breast.M = breast.M(dofPerm,:);
breast.M = breast.M(:,dofPerm);
breast.C = breast.C(dofPerm,:);
breast.C = breast.C(:,dofPerm);

%% Step 6: Import Reduced-Order Data

%% Uploading to simulation data
pathData = strcat(mainPath,"Simulink_models/Data/simulinkWorkSpace.mat");
save()

%% Auxiliar function
function C = computeModalDampingMatrix(dampingRatio,K,M)
% To avoid numerical issues (such as complex eigenvalues with very small
% imaginary parts), make the matrices exactly symmetric.

    K = (K+K')/2;    % Stiffness matrix
    M = (M+M')/2;    % Mass matrix

% Compute the eigen-decomposition associated with the mass and stiffness
% matrices, sorting the eigenvalues in ascending order and permuting
% the corresponding eigenvectors.

    [V,D] = eig(K,M);
    [d,sortIdxs] = sort(diag(D));
    V = V(:,sortIdxs);

% Due to small numerical errors, the six eigenvalues associated with the
% rigid-body modes may not be exactly zero. To avoid numerical issues,
% check that the first six eigenvalues are close enough to zero. Then
% replace them with exact 0 values.

    assert(all(abs(d(1:6))/abs(d(7)) < 1e-9),'Error due to "zero" eigenvalues.');
    d(1:6) = 0;

% Vectors of generalized masses and natural frequencies

    MV = M*V;
    generalizedMasses = diag(V'*MV);
    naturalFrequencies = sqrt(d);

% Compute the modal damping matrix associated with K and M

    C = MV * diag(2*dampingRatio*naturalFrequencies./generalizedMasses) * MV';

end

