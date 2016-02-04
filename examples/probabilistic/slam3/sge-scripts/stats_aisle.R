options(width = 300)
data = read.csv('data_new_aisle.csv')
attach(data)

data$time.step = data$time / data$steps;
data$ncells = data$dim;
data$errors = data$errors / (data$ncells - data$unknowns);
data$good = 100 * (1.0 - data$errors);
data$unknowns = 100 * (data$unknowns / data$ncells);

library(plyr)
data.stats = ddply(data, .(type, sir, ncells, dim, np, inference), summarize,
                   #ntrials = length(errors),
                   #steps.mean = round(mean(steps), 2),
                   #errors.mean = round(mean(errors), 2), errors.ci = round(qt(0.975, df = n-1) * sd(errors) / sqrt(n), 2),
                   good.mean = round(mean(good), 2), good.ci = round(qt(0.975, df = n-1) * sd(good) / sqrt(n), 2),
                   unknowns.mean = round(mean(unknowns), 2), unknowns.ci = round(qt(0.975, df = n-1) * sd(unknowns) / sqrt(n), 2),
                   time.step.mean = round(mean(time.step), 2), time.step.ci = round(qt(0.975, df = n-1) * sd(time.step) / sqrt(n), 2),
                   time.mean = round(mean(time), 2), time.ci = round(qt(0.975, df = n-1) * sd(time) / sqrt(n), 2))

#print(data.stats[1:10,])

# sort table
inference.order <- c("jt", "bp(maxtime=.5)", "bp(maxtime=.1)",
  "ac(kappa=.1,level=0,lazy=false,simple=false)", "ac(kappa=.1,level=0,lazy=false,simple=true)", "ac(kappa=.1,level=0,lazy=true)",
  "ac(kappa=.3,level=0,lazy=false,simple=false)", "ac(kappa=.3,level=0,lazy=false,simple=true)", "ac(kappa=.3,level=0,lazy=true)",
  "ac(kappa=.5,level=0,lazy=false,simple=false)", "ac(kappa=.5,level=0,lazy=false,simple=true)", "ac(kappa=.5,level=0,lazy=true)",
  "ac(kappa=.1,level=1,lazy=false,simple=false)", "ac(kappa=.1,level=1,lazy=false,simple=true)", "ac(kappa=.1,level=1,lazy=true)",
  "ac(kappa=.3,level=1,lazy=false,simple=false)", "ac(kappa=.3,level=1,lazy=false,simple=true)", "ac(kappa=.3,level=1,lazy=true)",
  "ac(kappa=.5,level=1,lazy=false,simple=false)", "ac(kappa=.5,level=1,lazy=false,simple=true)", "ac(kappa=.5,level=1,lazy=true)",
  "ac(kappa=.1,level=2,lazy=false,simple=false)", "ac(kappa=.1,level=2,lazy=false,simple=true)", "ac(kappa=.1,level=2,lazy=true)",
  "ac(kappa=.3,level=2,lazy=false,simple=false)", "ac(kappa=.3,level=2,lazy=false,simple=true)", "ac(kappa=.3,level=2,lazy=true)",
  "ac(kappa=.5,level=2,lazy=false,simple=false)", "ac(kappa=.5,level=2,lazy=false,simple=true)", "ac(kappa=.5,level=2,lazy=true)",
  "ac(kappa=.1,level=3,lazy=false,simple=false)", "ac(kappa=.1,level=3,lazy=false,simple=true)", "ac(kappa=.1,level=3,lazy=true)",
  "ac(kappa=.3,level=3,lazy=false,simple=false)", "ac(kappa=.3,level=3,lazy=false,simple=true)", "ac(kappa=.3,level=3,lazy=true)",
  "ac(kappa=.5,level=3,lazy=false,simple=false)", "ac(kappa=.5,level=3,lazy=false,simple=true)", "ac(kappa=.5,level=3,lazy=true)")

x <- transform(data.stats)
x.sorted <- x[order(x$type,x$sir,x$dim,x$np,match(x$inference, inference.order)),]
data.stats.sorted <- data.frame(x.sorted)

#print(data.stats.sorted[1:10,])

library(xtable)
#xt.peaked = xtable(data.stats[data.stats$type == "ore-slam-peaked", ])
xt.non.peaked.64 = xtable(data.stats.sorted[data.stats.sorted$type == "aisle-slam" & data.stats.sorted$dim == 64, 4:14])
xt.non.peaked.512 = xtable(data.stats.sorted[data.stats.sorted$type == "aisle-slam" & data.stats.sorted$dim == 512, 4:14])

#print(xt.peaked, include.rownames = FALSE)
print(xt.non.peaked.64, include.rownames = FALSE)
print(xt.non.peaked.512, include.rownames = FALSE)

