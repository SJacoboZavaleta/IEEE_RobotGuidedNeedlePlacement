# Calculating the biopsy success for a robot-guided needle placement
## Manuscript: Needle Placement for Robot-assisted 3D Ultrasound-guided Breast Biopsy: A Preliminary Study

## Importing libraries
library(readxl)
library(ggplot2)
library(plotly)
library(rcompanion)
library(DescTools)
library(ggthemes)
library(here)
### Uncomment only for using windows fonts like Times New Roman
# windowsFonts(Times=windowsFont("Times"))

## Importing data
### Getting positions and orientations after robot postplacement 
mainPath = dirname(dirname(here()))
dataPath = paste0(mainPath,"/MATLAB/Models/Simulink_models/Results/")

dataA_FNA = read_excel(paste0(dataPath,"simulationA_FNA.xls"), sheet = "Sheet3")
orientA_FNA = read_excel(paste0(dataPath,"simulationA_FNA.xls"), sheet = "Sheet4")
dataA_CN = read_excel(paste0(dataPath,"simulationA_CN.xls"), sheet = "Sheet3")
orientA_CN = read_excel(paste0(dataPath,"simulationA_CN.xls"), sheet = "Sheet4")
dataB_FNA = read_excel(paste0(dataPath,"simulationB_FNA.xls"), sheet = "Sheet3")
orientB_FNA = read_excel(paste0(dataPath,"simulationB_FNA.xls"), sheet = "Sheet4")
dataB_CN = read_excel(paste0(dataPath,"simulationB_CN.xls"), sheet = "Sheet3")
orientB_CN = read_excel(paste0(dataPath,"simulationB_CN.xls"), sheet = "Sheet4")
dataC_FNA = read_excel(paste0(dataPath,"simulationC_FNA.xls"), sheet = "Sheet3")
orientC_FNA = read_excel(paste0(dataPath,"simulationC_FNA.xls"), sheet = "Sheet4")
dataC_CN = read_excel(paste0(dataPath,"simulationC_CN.xls"), sheet = "Sheet3")
orientC_CN = read_excel(paste0(dataPath,"simulationC_CN.xls"), sheet = "Sheet4")
dataD_FNA = read_excel(paste0(dataPath,"simulationD_FNA.xls"), sheet = "Sheet3")
orientD_FNA = read_excel(paste0(dataPath,"simulationD_FNA.xls"), sheet = "Sheet4")
dataD_CN = read_excel(paste0(dataPath,"simulationD_CN.xls"), sheet = "Sheet3")
orientD_CN = read_excel(paste0(dataPath,"simulationD_CN.xls"), sheet = "Sheet4")

### Getting needle collisions before robot preplacement 
dataA_FNA2 = read_excel(paste0(dataPath,"simulationA_FNA.xls"), sheet = "Sheet2")
dataA_CN2 = read_excel(paste0(dataPath,"simulationA_CN.xls"), sheet = "Sheet2")
dataB_FNA2 = read_excel(paste0(dataPath,"simulationB_FNA.xls"), sheet = "Sheet2")
dataB_CN2 = read_excel(paste0(dataPath,"simulationB_CN.xls"), sheet = "Sheet2")
dataC_FNA2 = read_excel(paste0(dataPath,"simulationC_FNA.xls"), sheet = "Sheet2")
dataC_CN2 = read_excel(paste0(dataPath,"simulationC_CN.xls"), sheet = "Sheet2")
dataD_FNA2 = read_excel(paste0(dataPath,"simulationD_FNA.xls"), sheet = "Sheet2")
dataD_CN2 = read_excel(paste0(dataPath,"simulationD_CN.xls"), sheet = "Sheet2")

## For Cup A
posA_FNA = as.data.frame(dataA_FNA)
orientA_FNA = as.data.frame(orientA_FNA)
colnames(posA_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientA_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

posA_CN = as.data.frame(dataA_CN)
orientA_CN = as.data.frame(orientA_CN)
colnames(posA_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientA_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

### Checking unsuccessful reached targets due to robot kinematic limitation
table(posA_FNA$ePost)
table(orientA_FNA$ePost)
table(posA_CN$ePost)
table(orientA_CN$ePost)

table(posA_FNA$success)
table(orientA_FNA$success)
table(posA_CN$success)
table(orientA_CN$success)

# Choosing only experiments caused by robot intervention. For that, avoiding ones without success or outside maximum targeting errors

## CI
index = !(posA_FNA$success==0 & posA_FNA$ee=="[0,0,0]")
validPosA_FNA = posA_FNA$ePos[index]
validOrientA_FNA = orientA_FNA$ePos[index]

index = !(posA_CN$success==0 & posA_CN$ee=="[0,0,0]")
posValidA_CN = posA_CN$ePos[index]
validOrientA_CN = orientA_CN$ePos[index]

posA_FNA_CI =  MeanCI(validPosA_FNA, conf.level=0.95)
orientA_FNA_CI =  MeanCI(validOrientA_FNA, conf.level=0.95)
#t.test(validPosA_FNA,conf.level=0.95)

posA_CN_CI =  MeanCI(posValidA_CN, conf.level=0.95)
orientA_CN_CI =  MeanCI(validOrientA_CN, conf.level=0.95)

## For Cup B
posB_FNA = as.data.frame(dataB_FNA)
orientB_FNA = as.data.frame(orientB_FNA)
colnames(posB_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientB_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

posB_CN = as.data.frame(dataB_CN)
orientB_CN = as.data.frame(orientB_CN)
colnames(posB_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientB_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

table(posB_FNA$ePost)
table(orientB_FNA$ePost)
table(posB_CN$ePost)
table(orientB_CN$ePost)
table(posB_FNA$success)
table(posB_CN$success)

## CI
index = !(posB_FNA$success==0 & posB_FNA$ee=="[0,0,0]")
validPosB_FNA = posB_FNA$ePos[index]
validOrientaB_FNA = orientB_FNA$ePos[index]
index = !(posB_CN$success==0 & posB_CN$ee=="[0,0,0]")
validPosB_CN = posB_CN$ePos[index]
validOrientaB_CN = orientB_CN$ePos[index]

posB_FNA_CI =  MeanCI(validPosB_FNA, conf.level=0.95)
orientB_FNA_CI =  MeanCI(validOrientaB_FNA, conf.level=0.95)

posB_CN_CI =  MeanCI(validPosB_CN, conf.level=0.95)
orientB_CN_CI =  MeanCI(validOrientaB_CN, conf.level=0.95)

## For Cup C
posC_FNA = as.data.frame(dataC_FNA)
orientC_FNA = as.data.frame(orientC_FNA)
colnames(posC_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientC_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

posC_CN = as.data.frame(dataC_CN)
orientC_CN = as.data.frame(orientC_CN)
colnames(posC_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientC_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

table(posC_FNA$ePost)
table(orientC_FNA$ePost)
table(posC_CN$ePost)
table(orientC_CN$ePost)
table(posC_FNA$success)
table(posC_CN$success)

## CI
index = !(posC_FNA$success==0 & posC_FNA$ee=="[0,0,0]")
validPosC_FNA = posC_FNA$ePos[index]
validOrientaC_FNA = orientC_FNA$ePos[index]
index = !(posC_CN$success==0 & posC_CN$ee=="[0,0,0]")
validPosC_CN = posC_CN$ePos[index]
validOrientaC_CN = orientC_CN$ePos[index]

posC_FNA_CI =  MeanCI(validPosC_FNA, conf.level=0.95)
orientC_FNA_CI =  MeanCI(validOrientaC_FNA, conf.level=0.95)

posC_CN_CI =  MeanCI(validPosC_CN, conf.level=0.95)
orientC_CN_CI =  MeanCI(validOrientaC_CN, conf.level=0.95)

## For Cup D
posD_FNA = as.data.frame(dataD_FNA)
orientD_FNA = as.data.frame(orientD_FNA)
colnames(posD_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientD_FNA) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

posD_CN = as.data.frame(dataD_CN)
orientD_CN = as.data.frame(orientD_CN)
colnames(posD_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");
colnames(orientD_CN) = c("m","ee", "hs","ePre","needle","ns","ePost","success");

table(posD_FNA$ePost)
table(orientD_FNA$ePost)
table(posD_CN$ePost)
table(orientD_CN$ePost)
table(posD_FNA$success)
table(posD_CN$success)

index = !(posD_FNA$success==0 & posD_FNA$ee=="[0,0,0]")
validPosD_FNA = posD_FNA$ePos[index]
validOrientaD_FNA = orientD_FNA$ePos[index]
index = !(posD_CN$success==0 & posD_CN$ee=="[0,0,0]")
validPosD_CN = posD_CN$ePos[index]
validOrientaD_CN = orientD_CN$ePos[index]

posD_FNA_CI =  MeanCI(validPosD_FNA, conf.level=0.95)
orientD_FNA_CI =  MeanCI(validOrientaD_FNA, conf.level=0.95)

posD_CN_CI =  MeanCI(validPosD_CN, conf.level=0.95)
orientD_CN_CI =  MeanCI(validOrientaD_CN, conf.level=0.95)


# Reporting data
cupSize = c("A","A","B","B","C","C","D","D")
needleType = c("FNA","CN","FNA","CN","FNA","CN","FNA","CN")

## For position
meanErrors = round(c(posA_FNA_CI[1],posA_CN_CI[1],posB_FNA_CI[1],posB_CN_CI[1],
              posC_FNA_CI[1],posC_CN_CI[1],posD_FNA_CI[1],posD_CN_CI[1]),2)
minCI = round(c(posA_FNA_CI[2],posA_CN_CI[2],posB_FNA_CI[2],posB_CN_CI[2],
              posC_FNA_CI[2],posC_CN_CI[2],posD_FNA_CI[2],posD_CN_CI[2]),2)
maxCI = round(c(posA_FNA_CI[3],posA_CN_CI[3],posB_FNA_CI[3],posB_CN_CI[3],
              posC_FNA_CI[3],posC_CN_CI[3],posD_FNA_CI[3],posD_CN_CI[3]),2)

## Finding maximum error in either cases (success 1 or 0).
maxE = round(c(max(posA_FNA$ePost),max(posA_CN$ePost),max(posB_FNA$ePost),max(posB_CN$ePost),max(posC_FNA$ePost),max(posC_CN$ePost),max(posD_FNA$ePost),max(posD_CN$ePost)),2)

## Finding the minimum error only in successful cases (success==1)
minE = round(c(min(posA_FNA$ePost[posA_FNA$success==1]),min(posA_CN$ePost[posA_CN$success==1]),min(posB_FNA$ePost[posB_FNA$success==1]),min(posB_CN$ePost[posB_CN$success==1]),min(posC_FNA$ePost[posC_FNA$success==1]),min(posC_CN$ePost[posC_CN$success==1]),min(posD_FNA$ePost[posD_FNA$success==1]),min(posD_CN$ePost[posD_CN$success==1])),2)

meanPositionErrorsDF = data.frame(cupSize,needleType,meanErrors,minCI,maxCI,maxE,minE)
meanPositionErrorsDF

## For orientation
meanErrors = round(c(orientA_FNA_CI[1],orientA_CN_CI[1],orientB_FNA_CI[1],orientB_CN_CI[1],
              orientC_FNA_CI[1],orientC_CN_CI[1],orientD_FNA_CI[1],orientD_CN_CI[1])*180/3.1416,2)
minCI = round(c(orientA_FNA_CI[2],orientA_CN_CI[2],orientB_FNA_CI[2],orientB_CN_CI[2],
              orientC_FNA_CI[2],orientC_CN_CI[2],orientD_FNA_CI[2],orientD_CN_CI[2])*180/3.1416,2)
maxCI = round(c(orientA_FNA_CI[3],orientA_CN_CI[3],orientB_FNA_CI[3],orientB_CN_CI[3],
              orientC_FNA_CI[3],orientC_CN_CI[3],orientD_FNA_CI[3],orientD_CN_CI[3])*180/3.1416,2)

## Finding maximum error in either cases (success 1 or 0).
maxE = round(c(max(orientA_FNA$ePost),max(orientA_CN$ePost),max(orientB_FNA$ePost),max(orientB_CN$ePost),max(orientC_FNA$ePost),max(orientC_CN$ePost),max(orientD_FNA$ePost),max(orientD_CN$ePost))*180/3.1416,2)

## Finding the minimum error only in successful cases (success==1)
minE = round(c(min(orientA_FNA$ePost[orientA_FNA$success==1]),min(orientA_CN$ePost[orientA_FNA$success==1]),min(orientB_FNA$ePost[orientB_FNA$success==1]),min(orientB_CN$ePost[orientB_CN$success==1]),min(orientC_FNA$ePost[orientC_FNA$success==1]),min(orientC_CN$ePost[orientC_CN$success==1]),min(orientD_FNA$ePost[orientD_FNA$success==1]),min(orientD_CN$ePost[orientD_CN$success==1]))*180/3.1416,2)

meanOrientationErrorsDF = data.frame(cupSize,needleType,meanErrors,minCI,maxCI,maxE,minE)
meanOrientationErrorsDF

# Success of experiments by cup size considering all cases, even unreachable targets
successA_FNA = round(mean(posA_FNA$success)*100,1)
successA_CN = round(mean(posA_CN$success)*100,1)
successB_FNA = round(mean(posB_FNA$success)*100,1)
successB_CN = round(mean(posB_CN$success)*100,1)
successC_FNA = round(mean(posC_FNA$success)*100,1)
successC_CN = round(mean(posC_CN$success)*100,1)
successD_FNA = round(mean(posD_FNA$success)*100,1)
successD_CN = round(mean(posD_CN$success)*100,1)

successRate_DF = data.frame(cupSize,needleType,c(successA_FNA,successA_CN,successB_FNA,successB_CN,successC_FNA,successC_CN,successD_FNA,successD_CN))
colnames(successRate_DF) = c("cupSize","needleType","successRate_DF")
successRate_DF$cupSize = factor(successRate_DF$cupSize)
successRate_DF$needleType = factor(successRate_DF$needleType, levels = c("FNA","CN"))
successRate_DF

t1 = ggplot(successRate_DF, aes(x = cupSize, y=successRate_DF, fill = needleType)) + 
  geom_bar(stat = "identity",position=position_dodge(), colour="black")+
  theme_base(base_size = 12)+
  theme(legend.position = "top")+
  labs(fill = "Needle type")+
  geom_label(aes(label = successRate_DF, vjust = 2, hjust=+0.5), position = position_dodge(width= 0.9))+
  xlab("Breast holder size") +
  ylab("Biopsy Success")+
  scale_y_continuous(breaks = seq(0, 100, by = 10))
t1

ggsave(filename = "breastBiopsySuccess.eps", 
                plot = t1, 
                device = "eps", 
                dpi = 600, 
                width = 4.5,
                height = 3.5, 
                units = "in")

# Reporting needle collisions and success rate for each breast cup size

## Number of simulated experiments
lenA = 160
lenB = 137
lenC = 98
lenD = 89

successRate_DF$col1 = round(c(mean(dataA_FNA2$Colision1),mean(dataA_CN2$Colision1),mean(dataB_FNA2$Colision1),mean(dataB_CN2$Colision1),mean(dataC_FNA2$Colision1),mean(dataC_CN2$Colision1),mean(dataD_FNA2$Colision1),mean(dataD_CN2$Colision1))*100,1)
successRate_DF$col2Old = round(c(mean(dataA_FNA2$colision2Old),mean(dataA_CN2$colision2Old),mean(dataB_FNA2$colision2Old),mean(dataB_CN2$colision2Old),mean(dataC_FNA2$colision2Old),mean(dataC_CN2$colision2Old),mean(dataD_FNA2$colision2Old),mean(dataD_CN2$colision2Old))*100,1)
successRate_DF$col2 = round(c(mean(dataA_FNA2$colision2),mean(dataA_CN2$colision2),mean(dataB_FNA2$colision2),mean(dataB_CN2$colision2),mean(dataC_FNA2$colision2),mean(dataC_CN2$colision2),mean(dataD_FNA2$colision2),mean(dataD_CN2$colision2))*100,1)
successRate_DF$col3 = round(c(mean(dataA_FNA2$colision4),mean(dataA_CN2$colision4),mean(dataB_FNA2$colision4),mean(dataB_CN2$colision4),mean(dataC_FNA2$colision4),mean(dataC_CN2$colision4),mean(dataD_FNA2$colision4),mean(dataD_CN2$colision4))*100,1)
successRate_DF$col1inev = round(c(mean(dataA_FNA2$colision1Inev),mean(dataA_CN2$colision1Inev),mean(dataB_FNA2$colision1Inev),mean(dataB_CN2$colision1Inev),mean(dataC_FNA2$colision1Inev),mean(dataC_CN2$colision1Inev),mean(dataD_FNA2$colision1Inev),mean(dataD_CN2$colision1Inev))*100,1)
successRate_DF$joinLim = round(c(mean(dataA_FNA2$jointLimits),mean(dataA_CN2$jointLimits),mean(dataB_FNA2$jointLimits),mean(dataB_CN2$jointLimits),mean(dataC_FNA2$jointLimits),mean(dataC_CN2$jointLimits),mean(dataD_FNA2$jointLimits),mean(dataD_CN2$jointLimits))*100,1)
successRate_DF

# Plotting collision vs needle vs cup
# Creating a data frame for needle collisions
reportCollision = successRate_DF[,c(1,2,4)]
reportCollision$col = rep('Col1',8)
colnames(reportCollision) = c("cup","needle","col","nCol")

aux_c1 = successRate_DF[,c(1,2,5)]
aux_c1$col = rep('preCol2',8)
colnames(aux_c1) = c("cup","needle","col","nCol")
reportCollision = rbind(reportCollision,aux_c1)

aux_c2 = successRate_DF[,c(1,2,6)]
aux_c2$col = rep('Col2',8)
colnames(aux_c2) = c("cup","needle","col","nCol")
reportCollision = rbind(reportCollision,aux_c2)

aux_c3 = successRate_DF[,c(1,2,7)]
aux_c3$col = rep('Col3',8)
colnames(aux_c3) = c("cup","needle","col","nCol")
reportCollision = rbind(reportCollision,aux_c3)

aux_c4 = successRate_DF[,c(1,2,8)]
aux_c4$col = rep('iCol1',8)
colnames(aux_c4) = c("cup","needle","col","nCol")
reportCollision = rbind(reportCollision,aux_c4)

aux_c5 = successRate_DF[,c(1,2,9)]
aux_c5$col = rep('JointLim',8)
colnames(aux_c5) = c("cup","needle","col","nCol")
reportCollision = rbind(reportCollision,aux_c5)

reportCollision$nCol = factor(reportCollision$nCol,levels = c("Col1","iCol1","preCol2","Col2","Col3","JointLim"))


col <- ggplot(reportCollision, aes(x = cup, fill = nCol)) +
  geom_bar(aes(y = col), stat = "identity", position = position_dodge()) +
  scale_fill_brewer(palette = "Set3") +
  xlab("Cup size")+
  ylab("Collisions (%)")
  ggtitle("Collision types versus cup size")

col
 
col2 <- ggplot(reportCollision, aes(x = cup, y =col)) +
  geom_bar(aes(fill = nCol), stat = "identity") +
  scale_fill_brewer(palette = "Set3") +
  xlab("Cup size")+
  ylab("Collisions (%)")
  ggtitle("Collision types versus cup size")
col2


## Finding RMS for error positions and orientations
errorRMS <- function(error) 
{
   sqrt(mean(error^2))
}

RMS_posErrorA_FNA = round(errorRMS(validPosA_FNA),2)
RMS_posErrorA_CN = round(errorRMS(posValidA_CN),2)
RMS_orientErrorA_FNA = round(errorRMS(validOrientA_FNA),2)
RMS_orientErrorA_CN = round(errorRMS(validOrientA_CN),2)


RMS_posErrorB_FNA = round(errorRMS(validPosB_FNA),2)
RMS_posErrorB_CN = round(errorRMS(validPosB_CN),2)
RMS_orientErrorB_FNA = round(errorRMS(validOrientaB_FNA),2)
RMS_orientErrorB_CN = round(errorRMS(validOrientaB_CN),2)

RMS_posErrorC_FNA = round(errorRMS(validPosC_FNA),2)
RMS_posErrorC_CN = round(errorRMS(validPosC_CN),2)
RMS_orientErrorC_FNA = round(errorRMS(validOrientaC_FNA),2)
RMS_orientErrorC_CN = round(errorRMS(validOrientaC_CN),2)

RMS_posErrorD_FNA = round(errorRMS(validPosD_FNA),2)
RMS_posErrorD_CN = round(errorRMS(validPosD_CN),2)
RMS_orientErrorD_FNA = round(errorRMS(validOrientaD_FNA),2)
RMS_orientErrorD_CN = round(errorRMS(validOrientaD_CN),2)

RMSpos = c(RMS_posErrorA_FNA,RMS_posErrorA_CN,RMS_posErrorB_FNA,RMS_posErrorB_CN,RMS_posErrorC_FNA,RMS_posErrorC_CN,
           RMS_posErrorD_FNA,RMS_posErrorD_CN)
RMSorient = c(RMS_orientErrorA_FNA,RMS_orientErrorA_CN,RMS_orientErrorB_FNA,RMS_orientErrorB_CN,RMS_orientErrorC_FNA,RMS_orientErrorC_CN,
             RMS_orientErrorD_FNA,RMS_orientErrorD_CN)

RMS_pos = data.frame(cupSize, needleType,RMSpos,RMSorient)
RMS_pos


## Summary table
summaryBiopsy = cbind(meanPositionErrorsDF, RMSpos, meanOrientationErrorsDF[,c(-1,-2)],RMSorient)

## Box plotting

ValidErrorA = data.frame(error = c(validPosA_FNA,posValidA_CN))
ValidErrorA$cup = 'A'
ValidErrorA$needle = c(rep('FNA',length(validPosA_FNA)),rep('CN',length(posValidA_CN)))

ValidErrorB = data.frame(error = c(validPosB_FNA,validPosB_CN))
ValidErrorB$cup = 'B'
ValidErrorB$needle = c(rep('FNA',length(validPosB_FNA)),rep('CN',length(validPosB_CN)))

ValidErrorC = data.frame(error = c(validPosC_FNA,validPosC_CN))
ValidErrorC$cup = 'C'
ValidErrorC$needle = c(rep('FNA',length(validPosC_FNA)),rep('CN',length(validPosC_CN)))

ValidErrorD = data.frame(error = c(validPosD_FNA,validPosD_CN))
ValidErrorD$cup = 'D'
ValidErrorD$needle = c(rep('FNA',length(validPosD_FNA)),rep('CN',length(validPosD_CN)))

validPosError = rbind(ValidErrorA,ValidErrorB,ValidErrorC,ValidErrorD)
validPosError$cup = as.factor(validPosError$cup)
validPosError$needle = factor(validPosError$needle, levels = c("FNA","CN"))

t2 = ggplot(data=validPosError, aes(x=cup, y=error, color=needle))+
  stat_summary(fun = "mean", geom = "point", shape = 8,size = 3, position=position_dodge(width= 0.6),
               fun.args = list(conf.int = 0.95))+
  geom_boxplot(width = 0.6, outlier.shape=18, outlier.size=2)+
  stat_boxplot(geom = "errorbar", # Error bars
               width = 0.25) +    # Bars width
  geom_jitter(alpha = 0.1, position=position_jitter(0.1))+
  theme_base(base_size = 12)+
  theme(legend.position = "top")+
  labs(color = "Breast size")+
  xlab("Breast holder size") +
  ylab("Position Error (mm)")+
  scale_y_continuous(breaks = seq(1, 3, by = 1))
t2
#theme(legend.position = "top", axis.title = element_text(family = "Times"),title = element_text(family = "Times"))

ggsave(filename = "positionError.eps",
       plot = t2,
       device = "eps",
       dpi = 600,
       width = 2.6,
       height = 2.02,
       units = "in")

# For orientations
ValidErrorA = data.frame(error = c(validOrientA_FNA,validOrientA_CN))
ValidErrorA$cup = 'A'
ValidErrorA$needle = c(rep('FNA',length(validOrientA_FNA)),rep('CN',length(validOrientA_CN)))

ValidErrorB = data.frame(error = c(validOrientaB_FNA,validOrientaB_CN))
ValidErrorB$cup = 'B'
ValidErrorB$needle = c(rep('FNA',length(validOrientaB_FNA)),rep('CN',length(validOrientaB_CN)))

ValidErrorC = data.frame(error = c(validOrientaC_FNA,validOrientaC_CN))
ValidErrorC$cup = 'C'
ValidErrorC$needle = c(rep('FNA',length(validOrientaC_FNA)),rep('CN',length(validOrientaC_CN)))

ValidErrorD = data.frame(error = c(validOrientaD_FNA,validOrientaD_CN))
ValidErrorD$cup = 'D'
ValidErrorD$needle = c(rep('FNA',length(validOrientaD_FNA)),rep('CN',length(validOrientaD_CN)))

validOrientError = rbind(ValidErrorA,ValidErrorB,ValidErrorC,ValidErrorD)
validOrientError$cup = as.factor(validOrientError$cup)
validOrientError$needle = factor(validOrientError$needle, levels = c("FNA","CN"))


t3 = ggplot(data=validOrientError, aes(x=cup, y=error*180/3.1416,color=needle))+
  stat_summary(fun = "mean", geom = "point", shape = 8,size = 3, fun.args = list(conf.int = 0.95), position=position_dodge(width= 0.6))+
  geom_boxplot(width = 0.6, outlier.shape=18, outlier.size=2)+
  stat_boxplot(geom = "errorbar", # Error bars
               width = 0.25) +    # Bars width
  geom_jitter(alpha = 0.1,position=position_jitter(0.1))+
  scale_fill_manual(values=c("magenta", "#56B4E9", "green","orange"))+
  theme_base(base_size = 12)+
  theme(legend.position = "top")+
  labs(color = "Breast size")+
  xlab("Breast holder size") +
  ylab("Direction Error")+
  scale_y_continuous(breaks = seq(0, 5, by = 2.5))
t3

ggsave(filename = "directionError.eps",
       plot = t3,
       device = "eps",
       dpi = 600,
       width = 2.6,
       height = 2.02,
       units = "in")
