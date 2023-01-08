# Calculating the biopsy success for a robot-guided needle placement
## Manuscript: Needle Placement for Robot-assisted 3D Ultrasound-guided Breast Biopsy: A Preliminary Study

## Importing libraries
library(readxl)
library(ggplot2)
library(ggthemes)
library(latex2exp)
library(ggforce)
### Uncomment only for using windows fonts like Times New Roman
#windowsFonts(Times=windowsFont("Times"))

## Importing data
### Getting positions and orientations after robot postplacement 
mainPath = dirname(dirname(here()))
dataPath = paste0(mainPath,"/MATLAB/Models/Simulink_models/Results/")

vectorPosA_FNA = read_excel(paste0(mainPath,"simulationA_FNA.xls"), sheet = "Sheet5")
vectorPosA_CN = read_excel(paste0(mainPath,"simulationA_CN.xls"), sheet = "Sheet5")
vectorPosB_FNA = read_excel(paste0(mainPath,"simulationB_FNA.xls"), sheet = "Sheet5")
vectorPosB_CN = read_excel(paste0(mainPath,"simulationB_CN.xls"), sheet = "Sheet5")
vectorPosC_FNA = read_excel(paste0(mainPath,"simulationC_FNA.xls"), sheet = "Sheet5")
vectorPosC_CN = read_excel(paste0(mainPath,"simulationC_CN.xls"), sheet = "Sheet5")
vectorPosD_FNA = read_excel(paste0(mainPath,"simulationD_FNA.xls"), sheet = "Sheet5")
vectorPosD_CN = read_excel(paste0(mainPath,"simulationD_CN.xls"), sheet = "Sheet5")

# Plotting all reached targets inside breast volumes: Top view of breast holder device
## For Cup A
index = vectorPosA_FNA$Success==1
posTargetA_FNA = data.frame(Ps_x = vectorPosA_FNA$Ps_1[index], Ps_y =  vectorPosA_FNA$Ps_2[index], 
                            Ps_z =  vectorPosA_FNA$Ps_3[index])

index = vectorPosA_CN$Success==1
posTargetA_CN = data.frame(Ps_x =  vectorPosA_CN$Ps_1[index], Ps_y =  vectorPosA_CN$Ps_2[index], 
                           Ps_z =  vectorPosA_CN$Ps_3[index])

posTargetA = rbind(posTargetA_FNA,posTargetA_CN)
posTargetA$Cup = factor('A')

## For Cup B
index = vectorPosB_FNA$Success==1
posTargetB_FNA = data.frame(Ps_x =  vectorPosB_FNA$Ps_1[index], Ps_y =  vectorPosB_FNA$Ps_2[index], 
                            Ps_z =  vectorPosB_FNA$Ps_3[index])

index = vectorPosB_CN$Success==1
posTargetB_CN = data.frame(Ps_x =  vectorPosB_CN$Ps_1[index], Ps_y =  vectorPosB_CN$Ps_2[index], 
                           Ps_z =  vectorPosB_CN$Ps_3[index])

posTargetB = rbind(posTargetB_FNA,posTargetB_CN)
posTargetB$Cup = factor('B')

## For Cup C
index = vectorPosC_FNA$Success==1
posTargetC_FNA = data.frame(Ps_x =  vectorPosC_FNA$Ps_1[index], Ps_y =  vectorPosC_FNA$Ps_2[index], 
                            Ps_z =  vectorPosC_FNA$Ps_3[index])

index = vectorPosC_CN$Success==1
posTargetC_CN = data.frame(Ps_x =  vectorPosC_CN$Ps_1[index], Ps_y =  vectorPosC_CN$Ps_2[index], 
                           Ps_z =  vectorPosC_CN$Ps_3[index])

posTargetC = rbind(posTargetC_FNA,posTargetC_CN)
posTargetC$Cup = factor('C')

## For Cup D
index = vectorPosD_FNA$Success==1
posTargetD_FNA = data.frame(Ps_x =  vectorPosD_FNA$Ps_1[index], Ps_y =  vectorPosD_FNA$Ps_2[index], 
                            Ps_z =  vectorPosD_FNA$Ps_3[index])

index = vectorPosD_CN$Success==1
posTargetD_CN = data.frame(Ps_x =  vectorPosD_CN$Ps_1[index], Ps_y =  vectorPosD_CN$Ps_2[index], 
                           Ps_z =  vectorPosD_CN$Ps_3[index])

posTargetD = rbind(posTargetD_FNA,posTargetD_CN)
posTargetD$Cup = factor('D')

### Breast model dimensions
RA = 56.2
RB = 58.1
RC = 66.3
RD = 69.8
HA = 57.0
HB = 71.0
HC = 94.0
HD = 97.0

### Circumference of major radius of breast holder device and quadrant lines
hcA = 1100-posTargetA$Ps_z
lineVA <- data.frame(
  x = c(0,0),
  y = c(-RA,RA)
)
lineHA <- data.frame(
  y = c(0,0),
  x = c(-RA,RA)
)

hcB = 1100-posTargetB$Ps_z
lineVB <- data.frame(
  x = c(0,0),
  y = c(-RB,RB)
)
lineHB <- data.frame(
  y = c(0,0),
  x = c(-RB,RB)
)

hcC = 1100-posTargetC$Ps_z
lineVC <- data.frame(
  x = c(0,0),
  y = c(-RC,RC)
)
lineHC <- data.frame(
  y = c(0,0),
  x = c(-RC,RC)
)

hcD = 1100-posTargetD$Ps_z
lineVD <- data.frame(
  x = c(0,0),
  y = c(-RD,RD)
)
lineHD <- data.frame(
  y = c(0,0),
  x = c(-RD,RD)
)

### Some label positions
posXzoneA = c(12,15,-27,-10)
posYzoneA = c(45,-47,-37,40)

posXzoneB = c(20,15,-27,-10)
posYzoneB = c(45,-47,-37,47)

posXzoneC = c(20,15,-18,-15)
posYzoneC = c(45,-47,-37,54)

posXzoneD = c(20,30,-17,-20)
posYzoneD = c(45,-50,-55,20)

# Plotting x and y position against z of successful target points

## For Cup A
t4 = ggplot(posTargetA, aes(x=Ps_x, y=Ps_y, color=hcA)) + 
  geom_point(size=3) +  geom_circle(aes(x0 = 0, y0 = 0, r = RA),size=1.3,color='magenta',inherit.aes = FALSE)+
  scale_color_gradientn(colours = rainbow(n=4,start=1/6,end=4/6))+
  annotate("text", x = posXzoneA, y = posYzoneA, label = c("I", "IV","III","II"), colour = "red", size = 10)+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineVA, size=0.9, linetype="dashed")+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineHA, size=0.9, linetype="dashed")+
  theme_base(base_size = 16)+
  theme(legend.position = "bottom")+
  labs(color = TeX(r'(${c}_{z}\,(mm)$)'), x=TeX(r'(${c}_{x}\,(mm)$)'), y=TeX(r'(${c}_{y}\,(mm)$)'))

ggsave(filename = "targetsAvsXYZ.eps", 
       plot = t4, 
       device = "eps", 
       dpi = 600, 
       width = 4,
       height = 4.6, 
       units = "in")

## For Cup B
t5 = ggplot(posTargetB, aes(x=Ps_x, y=Ps_y, color=hcB)) + 
  geom_point(size=3) +  geom_circle(aes(x0 = 0, y0 = 0, r = RB),size=1.3,color='blue',inherit.aes = FALSE)+
  scale_color_gradientn(colours = rainbow(n=4,start=1/6,end=4/6))+
  annotate("text", x = posXzoneB, y = posYzoneB, label = c("I", "IV","III","II"), colour = "red", size = 10)+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineVB, size=0.9, linetype="dashed")+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineHB, size=0.9, linetype="dashed")+
  theme_base(base_size = 16)+
  theme(legend.position = "bottom")+
  labs(color = TeX(r'(${c}_{z}\,(mm)$)'), x=TeX(r'(${c}_{x}\,(mm)$)'), y=TeX(r'(${c}_{y}\,(mm)$)'))

ggsave(filename = "targetsBvsXYZ.svg", 
       plot = t5, 
       device = "svg", 
       dpi = 600, 
       width = 4,
       height = 4.6, 
       units = "in")

## For Cup C
t6 = ggplot(posTargetC, aes(x=Ps_x, y=Ps_y, color=hcC)) + 
  geom_point(size=3) +  geom_circle(aes(x0 = 0, y0 = 0, r = RC),size=1.3,color='green',inherit.aes = FALSE)+
  scale_color_gradientn(colours = rainbow(n=4,start=1/6,end=4/6))+
  annotate("text", x = posXzoneC, y = posYzoneC, label = c("I", "IV","III","II"), colour = "red", size = 10)+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineVC, size=0.9, linetype="dashed")+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineHC, size=0.9, linetype="dashed")+
  theme_base(base_size = 16)+
  theme(legend.position = "bottom", axis.title = element_text(family = "Times"),title = element_text(family = "Times"))+
  labs(color = TeX(r'(${c}_{z}\,(mm)$)'), x=TeX(r'(${c}_{x}\,(mm)$)'), y=TeX(r'(${c}_{y}\,(mm)$)'))

ggsave(filename = "targetsCvsXYZ.eps", 
       plot = t6, 
       device = "eps", 
       dpi = 600, 
       width = 4,
       height = 4.6, 
       units = "in")

## For Cup D
t7 = ggplot(posTargetD, aes(x=Ps_x, y=Ps_y, color=hcD)) + 
  geom_circle(aes(x0 = 0, y0 = 0, r = RD), colour="orange", size=1.3,inherit.aes = FALSE)+
  geom_point(size=3) +
  scale_color_gradientn(colours = rainbow(n=4,start=1/6,end=4/6))+
  annotate("text", x = posXzoneD, y = posYzoneD, label = c("I", "IV","III","II"), size = 10, colour = "red")+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineVD, size=0.9, linetype="dashed")+
  geom_line(aes(x=x, y=y), colour=c(1), data=lineHD, size=0.9, linetype="dashed")+
  theme_base(base_size = 16)+
  theme(legend.position = "bottom",axis.title = element_text(family = "Times"),title = element_text(family = "Times"))+
  labs(colour = TeX(r'(${c}_{z}\,(mm)$)'), x=TeX(r'(${c}_{x}\,(mm)$)'), y=TeX(r'(${c}_{y}\,(mm)$)'))

ggsave(filename = "targetsDvsXYZ.eps", 
       plot = t7, 
       device = "eps", 
       dpi = 600, 
       width = 4,
       height = 4.6, 
       units = "in")