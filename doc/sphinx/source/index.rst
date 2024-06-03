.. OpenLoong project documentation master file, created by
   sphinx-quickstart on Wed Apr 10 2024.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. toctree::
   :glob:
   :hidden:
   :maxdepth: 2
   :caption: Tutorial

   Tutorial/Install.md
   Tutorial/Develop.md
   Tutorial/Basic Knowledge.rst

.. toctree::
   :glob:
   :hidden:
   :maxdepth: 2
   :caption: API

   API/Algorithm/index.rst
   API/Common/index.rst

OpenLoong Dynamics Control Project
===================================

Welcome to OpenLoong Dynamics Control Project's documentation!
---------------------------------------------------------------

OpenLoong Dynamics Control Project is an open-source project led by Humanoid Robot (Shanghai) Co., Ltd, Shanghai Humanoid Robot Manufacturing Innovation Center and OpenAtom Foundation.

This project provides a MPC-WBC based control framework based on the "AzureLoong" robot model in Shanghai Humanoid Robotics Innovation Center, which runs on Mujoco and shows three examples including walking, jumping and blindly stepping over obstales. `Project Homepage <http://aa.com>`_

Attribute
*********

**Easy to Deploy**: This project provides a comprehensive solution for humanoid control. The code is self-contained so that users can easily configure their working environment and run without installing other dependencies to simplify the deploying process.

**Easy to Understand**: The humanoid control framework structure adopts a hierarchical modular design, improving the maintainability and expandability of the system. The design draws clear boundaries for all the modules of the system in terms of logic and function, which provides a more friendly environment for secondary development and enables the developers to customize and expand the functions more easily.

**Easy to Develop**: The code structure is simple, following the code design principle of module encapsulation for function, applying bus for data interaction between modules, reducing encapsulation redundancy, and helping to reduce code complexity; algorithm implementation adopts the simple logic of "read-calculate-write", which improves the readability of the code.

.. image:: ../../resources/images/figure1.png
   :scale: 40 %



Indices and tables
==================

* :ref:`genindex`


.. * :ref:`modindex`
.. * :ref:`search`
