options(width=300)
data=read.csv('data2.csv')
attach(data)

library(plyr)
ddply(data, .(type,sir,dim,np,inference), summarize, mean.error=round(mean(errors),2), sd.error=round(sd(errors),2), mean.unknown=round(mean(unknowns),2), sd.unknown=round(sd(unknowns),2), mean.time=round(mean(time),2), sd.time=round(sd(time),2))

