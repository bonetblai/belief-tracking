options(width = 300)
data = read.csv('data.csv')
attach(data)

library(plyr)
data.stats = ddply(data, .(type, sir, dim, np, inference), summarize,
                   ntrials  =length(errors),
                   steps.mean = round(mean(steps), 2),
                   errors.mean = round(mean(errors), 2), errors.ci = round(qt(0.975, df = n-1) * sd(errors) / sqrt(n), 2),
                   unknowns.mean = round(mean(unknowns), 2), unknowns.ci = round(qt(0.975, df = n-1) * sd(unknowns) / sqrt(n), 2),
                   time.mean = round(mean(time), 2), time.ci = round(qt(0.975, df = n-1) * sd(time) / sqrt(n), 2))

library(xtable)
xt.peaked = xtable(data.stats[data.stats$type == "ore-slam-peaked", ])
xt.non.peaked = xtable(data.stats[data.stats$type == "ore-slam-non-peaked", ])

print(xt.peaked, include.rownames = FALSE)
print(xt.non.peaked, include.rownames = FALSE)

