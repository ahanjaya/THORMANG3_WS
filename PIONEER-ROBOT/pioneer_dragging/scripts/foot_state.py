#! /usr/bin/env python3

import matplotlib.pyplot as plt

# style_plot = random.choice(plt.style.available)
style_plot = 'seaborn-deep'#'fast'

plt.style.use(style_plot)
plt.rcParams.update({'font.size': 28})
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"

def create_figure(figure_n, title, x_label, y_label):
    fig = plt.figure(figure_n, figsize=(12,8))
    ax1  = fig.add_subplot(2,1,1)
    ax2  = fig.add_subplot(2,1,2)

    ax1.set_xlabel(x_label)
    ax1.set_ylabel(y_label)
    ax2.set_xlabel(x_label)
    ax2.set_ylabel(y_label)
    return fig, (ax1, ax2)


fig5, (ax5, ax6) = create_figure(figure_n=5, title='F/T Sensor', x_label='Step', y_label='Sate')
l_foot = [1, 0, 1, 0, 1, 0, 1, 0, 1]
r_foot = [1, 1, 0, 1, 0, 1, 0, 1, 0]

color_green  = 'tab:green'
color_red    = 'tab:red'

ax5.step(l_foot, '-', color=color_red,    label='Left Foot')
ax6.step(r_foot, '-', color=color_green,  label='Right Foot')

ax5.grid()
ax5.legend()

ax6.grid()
ax6.legend()

plt.tight_layout()
plt.show(block=False)
input('Press enter')
plt.close('all')