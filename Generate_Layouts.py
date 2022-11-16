import random

Agents = 20
Num_Layouts_Per = 10

layout1 = ["9 22\n",
           ". . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . . \n",
           ". . . . . . . . . . . . . . . . . . . . . . \n",
           ". . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . . \n",
           ". . . . . . . . . . . . . . . . . . . . . . \n",
           ". . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . . \n",
           ". . . . . . . . . . . . . . . . . . . . . . \n",
           ". . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . . \n",
           ". . . . . . . . . . . . . . . . . . . . . . \n",
           ". . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . . \n"]

locs1 = []
for k in range(1, 10):
    for p in range(22):
        if layout1[k][2*p] != '@':
            locs1.append([k-1, p])
print(locs1)

layout2 = ['9 22\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n']

locs2 = []
for k in range(1, 10):
    for p in range(22):
        if layout2[k][2*p] != '@':
            locs2.append([k-1, p])
print(locs2)

layout3 = ['9 22\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . @ . @ @ @ @ . @ . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . @ . @ @ @ @ . @ . @ @ @ @ . .\n',
           '. . . . . . . @ . . . . . . @ . . . . . . .\n',
           '. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .\n']
locs3 = []
for k in range(1, 10):
    for p in range(22):
        if layout3[k][2*p] != '@':
            locs3.append([k-1, p])
print(locs3)

for i in range(1, Num_Layouts_Per + 1):
    agents = []
    points = random.sample(locs1, 2*Agents)
    agentStarts = points[:Agents]
    agentGoals = points[Agents:]
    for j in range(Agents-1):
        agents.append(str(agentStarts[j][0]) + ' ' + str(agentStarts[j][1]) + ' ' + str(agentGoals[j][0]) + ' ' + str(
            agentGoals[j][1]) + '\n')
    agents.append(
        str(agentStarts[j + 1][0]) + ' ' + str(agentStarts[j + 1][1]) + ' ' + str(agentGoals[j + 1][0]) + ' ' + str(
            agentGoals[j + 1][1]))
    f = open("layouts/layout1/layout1_" + str(Agents) + 'agents_' + str(i) + ".txt", 'w')
    f.writelines(layout1)
    f.write(str(Agents) + '\n')
    f.writelines(agents)
    f.close()

    agents = []
    points = random.sample(locs2, 2 * Agents)
    agentStarts = points[:Agents]
    agentGoals = points[Agents:]
    for j in range(Agents-1):
        agents.append(str(agentStarts[j][0]) + ' ' + str(agentStarts[j][1]) + ' ' + str(agentGoals[j][0]) + ' ' + str(
            agentGoals[j][1]) + '\n')
    agents.append(
        str(agentStarts[j + 1][0]) + ' ' + str(agentStarts[j + 1][1]) + ' ' + str(agentGoals[j + 1][0]) + ' ' + str(
            agentGoals[j + 1][1]))
    f = open("layouts/layout2/layout2_" + str(Agents) + 'agents_' + str(i) + ".txt", 'w')
    f.writelines(layout2)
    f.write(str(Agents) + '\n')
    f.writelines(agents)
    f.close()

    agents = []
    points = random.sample(locs3, 2 * Agents)
    agentStarts = points[:Agents]
    agentGoals = points[Agents:]
    for j in range(Agents-1):
        agents.append(str(agentStarts[j][0]) + ' ' + str(agentStarts[j][1]) + ' ' + str(agentGoals[j][0]) + ' ' + str(
            agentGoals[j][1]) + '\n')
    agents.append(str(agentStarts[j+1][0]) + ' ' + str(agentStarts[j+1][1]) + ' ' + str(agentGoals[j+1][0]) + ' ' + str(
            agentGoals[j+1][1]))
    f = open("layouts/layout3/layout3_" + str(Agents) + 'agents_' + str(i) + ".txt", 'w')
    f.writelines(layout3)
    f.write(str(Agents) + '\n')
    f.writelines(agents)
    f.close()
