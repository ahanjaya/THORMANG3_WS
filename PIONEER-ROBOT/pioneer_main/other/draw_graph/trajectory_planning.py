#!/usr/bin/env python3

import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D


def plots_(axes_, x_, y_, label_=None, xlabel_="", ylabel_="", title_=""):
    # axes_.plot(x_, y_, 'o-', label=label_)
    axes_.plot(x_, y_, label=label_)
    # axes_.set_xlabel(xlabel_)
    # axes_.set_ylabel(ylabel_)
    # axes_.set_title(title_)
    # axes_.grid()

def trajectory_sin(time, res):
    # fig1, axes = plt.subplots(nrows=3, ncols=1, figsize=(8,8))
    fig1, axes = plt.subplots(nrows=1, ncols=1)

    # Axes 0
    t = np.linspace(0.0, np.pi, num=time/res)
    s = np.sin(t) #* 0
    # plots_(axes[0], t, s, None, "theta (${\Theta}$)", "sin (${\Theta}$)", "Trajectory Planning - Sinus")

    # Axes1
    # total_time = np.linspace(0.0, time, num=time/res)
    # plots_(axes[0], total_time, total_time*0, None, "time")

    x_distance = np.linspace(0.1, 0.5, num=time/res)
    y_distance = np.ones(int(time/res)) * 0.4
    plots_(axes, x_distance, y_distance, "Direct", "x axis", "y axis", "Sinus Trajectory")

    # Axes2
    x_dis  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y_dis  = np.interp(s, [0, 1],     [0.4, 0.5])
    y_dis1 = np.interp(s, [0, 1],     [0.4, 0.3])
    # y_dis = h_curve * s
    plots_(axes, x_dis, y_dis, "Sin", "x axis", "y axis") #, "Sinus Trajectory")
    plots_(axes, x_dis, y_dis1, "- Sin", "x axis", "y axis")
    axes.set_xlabel("y axis (m)")
    axes.set_ylabel("z axis (m)")
    axes.set_title("Sin Trajectory Planning")
    axes.grid()

    axes.legend()
    # fig1.tight_layout()

def gaussian(x, mu, sig):
    return np.exp( -np.power(x - mu, 2.) / (2 * np.power(sig, 2.)) )

def normal_gaussian(x, mu, sig):
    return ( 1 / (sig * np.sqrt(2*np.pi)) ) * np.exp( -np.power(x - mu, 2.) / (2 * np.power(sig, 2.)) )

def trajectory_gaussian(time, res):
    # x_values = np.linspace(-5, 5, num=time/res)
    x_values = np.linspace(-5, 5, num=time/res)
    fig2, axes = plt.subplots(nrows=2, ncols=1)

    # # Axes1
    # total_time = np.linspace(0.0, time, num=time/res)
    # plots_(axes[1], total_time, total_time*0, None, "time")

    # Axes2
    # x_dis = np.interp(x_values, [-5, 5], [20, 40])
    x_dis = np.interp(x_values, [-5, 5], [0.1, 0.5])

    max_gauss = []

    for mu, sig in [(0, 0.8), (0, 1.0), (-2, 1.2), (2, 0.5)]:
        y_values = norm.pdf(x_values, mu, sig)
        max_gauss.append(np.max(y_values))
    
    max_gauss = np.array(max_gauss)
    y_max = np.interp(max_gauss, [0, np.max(max_gauss)], [0.4, 0.5])

    i = 0
    for mu, sig in [(0, 0.8), (0, 1.0), (-2, 1.2), (2, 0.5)]:
        label = "$\mu$ = {0}, $\sigma$ = {1}".format(mu, sig) 
        # plots_(axes, x_values, gaussian(x_values, mu, sig), label, r'X', r'$\varphi _{\mu ,\sigma ^{2}} = (X)$', "Trajectory Gaussian")
        # plots_(axes, x_values, normal_gaussian(x_values, mu, sig), label, r'X', r'$\varphi _{\mu ,\sigma ^{2}} = (X)$', "Trajectory Gaussian")

        y_values = norm.pdf(x_values, mu, sig)
        plots_(axes[0], x_values, y_values, label, r'X', r'$\varphi _{\mu ,\sigma ^{2}} = (X)$', "Trajectory Gaussian")

        # Axes2
        # y_dis = np.interp(y_values, [0, np.max(y_values)], [0.4, 0.5])
        y_dis = np.interp(y_values, [0, np.max(y_values)], [0.4, y_max[i]])

        plots_(axes[1], x_dis, y_dis, None, "x distance", "y distance")
        i +=1

    fig2.legend()


def sin_single_axis():
    time = 2
    res  = 0.1
    
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure(figsize=(7, 6))
    ax = fig.gca(projection='3d')
    
    t = np.linspace(0.0, np.pi, num=time/res)
    s = np.sin(t)

    x  = np.linspace(0.1, 0.5, num=time/res)
    y  = np.ones(int(time/res)) * 0.4
    z  = np.ones(int(time/res)) * 0.8
    ax.plot(x, y, z, label='Direct')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.interp(s, [0, 1],     [0.4, 0.5])
    z  = np.ones(int(time/res)) * 0.8
    ax.plot(x, y, z, label='Sin X')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.interp(s, [0, 1],     [0.4, 0.3])
    z  = np.ones(int(time/res)) * 0.8
    ax.plot(x, y, z, label='- Sin X')


    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.ones(int(time/res)) * 0.4
    z  = np.interp(s, [0, 1],     [0.8, 0.9])
    ax.plot(x, y, z, label='Sin Z')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.ones(int(time/res)) * 0.4
    z  = np.interp(s, [0, 1],     [0.8, 0.7])
    ax.plot(x, y, z, label='- Sin Z')

    ax.plot([0.1], [0.4], [0.8], marker='o', markerfacecolor='k',)
    ax.plot([0.5], [0.4], [0.8], marker='o', markerfacecolor='k',)

    ax.set_xlabel("y axis (m)")
    ax.set_ylabel("x axis (m)")
    ax.set_zlabel("z axis (m)")
    ax.set_xlim(0.1, 0.5)
    ax.set_ylim(0.3, 0.5)
    ax.set_zlim(0.7, 0.9)
    ax.set_title("Sin Trajectory Planning")
    # ax.legend(loc='center right')
    ax.legend()
    ax.view_init(azim=45)

    fig.tight_layout()

def sin_omni_axis():
    time = 2
    res  = 0.1
    
    fig = plt.figure(figsize=(7, 6))
    ax = fig.gca(projection='3d')
    
    t = np.linspace(0.0, np.pi, num=time/res)
    s = np.sin(t)

    x  = np.linspace(0.1, 0.5, num=time/res)
    y  = np.ones(int(time/res)) * 0.4
    z  = np.ones(int(time/res)) * 0.8
    ax.plot(x, y, z, label='Direct')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.interp(s, [0, 1],     [0.4, 0.5])
    z  = np.interp(s, [0, 1],     [0.8, 0.9])
    ax.plot(x, y, z, label='Sin X & Sin Z')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.interp(s, [0, 1],     [0.4, 0.3])
    z  = np.interp(s, [0, 1],     [0.8, 0.9])
    ax.plot(x, y, z, label='- Sin X & Sin Z')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.interp(s, [0, 1],     [0.4, 0.3])
    z  = np.interp(s, [0, 1],     [0.8, 0.7])
    ax.plot(x, y, z, label='- Sin X & - Sin Z')

    x  = np.interp(t, [0, np.pi], [0.1, 0.5])
    y  = np.interp(s, [0, 1],     [0.4, 0.5])
    z  = np.interp(s, [0, 1],     [0.8, 0.7])
    ax.plot(x, y, z, label='Sin X & - Sin Z')

    ax.plot([0.1], [0.4], [0.8], marker='o', markerfacecolor='k',)
    ax.plot([0.5], [0.4], [0.8], marker='o', markerfacecolor='k',)

    ax.set_xlabel("y axis (m)")
    ax.set_ylabel("x axis (m)")
    ax.set_zlabel("z axis (m)")
    ax.set_xlim(0.1, 0.5)
    ax.set_ylim(0.3, 0.5)
    ax.set_zlim(0.7, 0.9)
    ax.set_title("Sin Trajectory Planning")
    # ax.legend(loc='center right')
    ax.legend()
    ax.view_init(azim=45)
    ax.view_init(azim=45)

    fig.tight_layout()

def test_gauss(time, res):
    x_values = np.linspace(-5, 5, num=time/res)
    # x_values = np.linspace(-5, 5, num=200)
    fig2, axes = plt.subplots(nrows=1, ncols=1)

    x_dis = np.interp(x_values, [-5, 5], [0.1, 0.5])

    max_gauss = []

    for mu, sig in [(0, 0.8), (0, 1.0), (-2, 0.5), (2, 1.2)]:
        y_values = norm.pdf(x_values, mu, sig)
        max_gauss.append(np.max(y_values))
    
    max_gauss = np.array(max_gauss)
    y_max = np.interp(max_gauss, [0, np.max(max_gauss)], [0.4, 0.5])

    i = 0

    for mu, sig in [(0, 0.8), (0, 1.0), (-2, 0.5), (2, 1.2)]:
        label = "$\mu$ = {0}, $\sigma$ = {1}".format(mu, sig) 
        y_values = norm.pdf(x_values, mu, sig)

        # Axes2
        # y_dis = np.interp(y_values, [0, np.max(y_values)], [10, 30])
        y_dis = np.interp(y_values, [0, np.max(y_values)], [0.4, y_max[i]])

        plots_(axes, x_dis, y_dis, label, "x distance", "y distance")
        i += 1

    axes.grid()
    axes.set_xlabel("y axis (m)")
    axes.set_ylabel("z axis (m)")
    axes.set_title("Gaussian Trajectory Planning")
    axes.legend()
    

def test_gauss1(time, res):
    fig = plt.figure(figsize=(7, 6))
    ax = fig.gca(projection='3d')

    x_values = np.linspace(-5, 5, num=time/res)
    x  = np.interp(x_values, [-5, 5], [0.1, 0.5])
    y  = np.ones(int(time/res)) * 0.4
    mu = -2.0
    sig = 0.5
    label = "$\mu$ = {0}, $\sigma$ = {1} (Z)".format(mu, sig)
    z_values = norm.pdf(x_values, mu, sig)
    z = np.interp(z_values, [0, np.max(z_values)], [0.8, 0.7])
    ax.plot(x, y, z, label=label)

    mu = 2.0
    sig = 0.5
    label = "$\mu$ = {0}, $\sigma$ = {1} (Z)".format(mu, sig)
    z_values = norm.pdf(x_values, mu, sig)
    z = np.interp(z_values, [0, np.max(z_values)], [0.8, 0.9])
    ax.plot(x, y, z, label=label)

    mu = -2.0
    sig = 0.5
    label = "$\mu$ = {0}, $\sigma$ = {1} (X)".format(mu, sig)
    z  = np.ones(int(time/res)) * 0.8
    y_values = norm.pdf(x_values, mu, sig)
    y = np.interp(y_values, [0, np.max(y_values)], [0.4, 0.3])
    ax.plot(x, y, z, label=label)

    mu = 2.0
    sig = 0.5
    label = "$\mu$ = {0}, $\sigma$ = {1} (X)".format(mu, sig)
    z  = np.ones(int(time/res)) * 0.8
    y_values = norm.pdf(x_values, mu, sig)
    y = np.interp(y_values, [0, np.max(y_values)], [0.4, 0.5])
    ax.plot(x, y, z, label=label)


    ax.plot([0.1], [0.4], [0.8], marker='o', markerfacecolor='k',)
    ax.plot([0.5], [0.4], [0.8], marker='o', markerfacecolor='k',)

    ax.set_xlabel("y axis (m)")
    ax.set_ylabel("x axis (m)")
    ax.set_zlabel("z axis (m)")
    ax.set_xlim(0.1, 0.5)
    ax.set_ylim(0.3, 0.5)
    ax.set_zlim(0.7, 0.9)
    ax.set_title("Gaussian Trajectory Planning")
    # ax.legend(loc='center right')
    ax.legend()
    ax.view_init(azim=65)

    fig.tight_layout()


def main():
    time = 2
    res  = 0.01
    trajectory_sin(time, res)
    sin_single_axis()
    sin_omni_axis()
    # test_gauss(time, res)
    # test_gauss1(time, res)
    # trajectory_gaussian(time, res)

    # plt.show()
    plt.show(block=False)
    plt.pause(0.1)
    input("Press [enter] to close.")
    plt.close('all')

if __name__ == '__main__':
    main()