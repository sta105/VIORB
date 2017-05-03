import numpy as np
import matplotlib.pyplot as plt

filepath = '/home/jp/opensourcecode/ORB_SLAM2/tmp/';

errlbapvr1 = np.loadtxt(filepath+'dbgLBAPVRErr.txt');
#errlbapvr1 = np.loadtxt(filepath+'osdbgLBAPVRErr.txt');
plt.figure(1);
p11, =plt.plot(errlbapvr1[:,0]);
p12, =plt.plot(errlbapvr1[:,1]);
p13, =plt.plot(errlbapvr1[:,2]);
plt.title('dbgLBAPVRErr1-P');
plt.legend([p11,p12,p13],["x","y","z"]);
plt.figure(2);
p21, =plt.plot(errlbapvr1[:,3]);
p22, =plt.plot(errlbapvr1[:,4]);
p23, =plt.plot(errlbapvr1[:,5]);
plt.title('dbgLBAPVRErr1-V');
plt.legend([p21,p22,p23],["x","y","z"]);
plt.figure(3);
p31, =plt.plot(errlbapvr1[:,6]);
p32, =plt.plot(errlbapvr1[:,7]);
p33, =plt.plot(errlbapvr1[:,8]);
plt.title('dbgLBAPVRErr1-R');
plt.legend([p31,p32,p33],["x","y","z"]);

errlbabias1 = np.loadtxt(filepath+'dbgLBABiasErr.txt');
#errlbabias1 = np.loadtxt(filepath+'osdbgLBABiasErr.txt');
plt.figure(7);
p71, =plt.plot(errlbabias1[:,0]);
p72, =plt.plot(errlbabias1[:,1]);
p73, =plt.plot(errlbabias1[:,2]);
plt.title('dbgLBABiasErr1-gyr');
plt.legend([p71,p72,p73],["x","y","z"]);
plt.figure(8);
p81, =plt.plot(errlbabias1[:,3]);
p82, =plt.plot(errlbabias1[:,4]);
p83, =plt.plot(errlbabias1[:,5]);
plt.title('dbgLBABiasErr1-acc');
plt.legend([p81,p82,p83],["x","y","z"]);

errlbapvr2 = np.loadtxt(filepath+'dbgLBAPVRErr2.txt');
plt.figure(4);
p41, =plt.plot(errlbapvr2[:,0]);
p42, =plt.plot(errlbapvr2[:,1]);
p43, =plt.plot(errlbapvr2[:,2]);
plt.title('dbgLBAPVRErr2-P');
plt.legend([p41,p42,p43],["x","y","z"]);
plt.figure(5);
p51, =plt.plot(errlbapvr2[:,3]);
p52, =plt.plot(errlbapvr2[:,4]);
p53, =plt.plot(errlbapvr2[:,5]);
plt.title('dbgLBAPVRErr2-V');
plt.legend([p51,p52,p53],["x","y","z"]);
plt.figure(6);
p61, =plt.plot(errlbapvr2[:,6]);
p62, =plt.plot(errlbapvr2[:,7]);
p63, =plt.plot(errlbapvr2[:,8]);
plt.title('dbgLBAPVRErr2-R');
plt.legend([p61,p62,p63],["x","y","z"]);

errlbabias2 = np.loadtxt(filepath+'dbgLBABiasErr2.txt');
plt.figure(9);
p91, =plt.plot(errlbabias2[:,0]);
p92, =plt.plot(errlbabias2[:,1]);
p93, =plt.plot(errlbabias2[:,2]);
plt.title('dbgLBABiasErr2-gyr');
plt.legend([p91,p92,p93],["x","y","z"]);
plt.figure(10);
p101, =plt.plot(errlbabias2[:,3]);
p102, =plt.plot(errlbabias2[:,4]);
p103, =plt.plot(errlbabias2[:,5]);
plt.title('dbgLBABiasErr2-acc');
plt.legend([p101,p102,p103],["x","y","z"]);


errpoptpvr1 = np.loadtxt(filepath+'dbg1fPoseOptPVRErr.txt');
plt.figure(11);
p111, =plt.plot(errpoptpvr1[:,0]);
p112, =plt.plot(errpoptpvr1[:,1]);
p113, =plt.plot(errpoptpvr1[:,2]);
plt.title('dbg1fPoseOptPVRErr-P');
plt.figure(12);
p121, =plt.plot(errpoptpvr1[:,3]);
p122, =plt.plot(errpoptpvr1[:,4]);
p123, =plt.plot(errpoptpvr1[:,5]);
plt.title('dbg1fPoseOptPVRErr-V');
plt.figure(13);
p131, =plt.plot(errpoptpvr1[:,6]);
p132, =plt.plot(errpoptpvr1[:,7]);
p133, =plt.plot(errpoptpvr1[:,8]);
plt.title('dbg1fPoseOptPVRErr-R');

errpoptbias1 = np.loadtxt(filepath+'dbg1fPoseOptBiasErr.txt');
plt.figure(14);
p141, =plt.plot(errpoptbias1[:,0]);
p142, =plt.plot(errpoptbias1[:,1]);
p143, =plt.plot(errpoptbias1[:,2]);
plt.title('dbg1fPoseOptBiasErr-gyr');
plt.figure(15);
p151, =plt.plot(errpoptbias1[:,3]);
p152, =plt.plot(errpoptbias1[:,4]);
p153, =plt.plot(errpoptbias1[:,5]);
plt.title('dbg1fPoseOptBiasErr-acc');

errpoptprior1 = np.loadtxt(filepath+'dbg1fPoseOptPriorErr.txt');
plt.figure(17);
p171, =plt.plot(errpoptprior1[:,0]);
p172, =plt.plot(errpoptprior1[:,1]);
p173, =plt.plot(errpoptprior1[:,2]);
plt.title('dbg1fPoseOptPriorErr-P');
plt.figure(18);
p181, =plt.plot(errpoptprior1[:,3]);
p182, =plt.plot(errpoptprior1[:,4]);
p183, =plt.plot(errpoptprior1[:,5]);
plt.title('dbg1fPoseOptPriorErr-V');
plt.figure(19);
p191, =plt.plot(errpoptprior1[:,6]);
p192, =plt.plot(errpoptprior1[:,7]);
p193, =plt.plot(errpoptprior1[:,8]);
plt.title('dbg1fPoseOptPriorErr-R');

plt.show();
