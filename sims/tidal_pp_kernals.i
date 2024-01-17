# altering tidal_pp_fullySat to use the kernals instead of the action
# added kernals based on test/pp_generation.i
# porepressure increases linearly like in pp generation
# not cyclically like expected ...
# that was because i added all earth gravity
# without earth gravity it performs well
# still requires to increase nl_abs_tol for convergence
# matches exactly with PorousFlowBasicTHM now :D

# does not converge with PP BC and BodyForce/gravity applied
# Nonlinear solve did not converge due to DIVERGED_DTOL iterations 2
#  Solve Did NOT Converge!
# Aborting as solve did not converge
# need to calculate gravity for enceladus
# do i need nodalgravity?

# added gravity for x, y, z
# cannot get mesh to converge (works okay for 1)

# using SVD
# shows x of y singular values are zero which might be
# bad boundary conditions
# added pin on "pole" to have one more BC
# can try just fixing one dimension of displacement

# shows high condition number
# suggests preconditionin, scaling, mesh refinement
# increased number of nodes to 20 x 20 x20
# runs way too slow
# went to 10 x 10 x 10
# same issue with convergence
#

# working on coupling of HydroMechanical
# i think i was using the wrong strain/stress model
# moved from "SMALL" to "FINITE"
# does not converge
# tried SVD but takes too long
# turned off all preconditioning and svd and IterationAdaptiveDT
# lots of linear iterates and it often doesn't converge even
# after l_max_its gets down to about 7.9e-10
# adding smp back in after reading that failed linear solve
# is due to bad preconditioning
# didn't work, l res is 6.8e-10 so it is lower
# trying mumps
# seems to work better but DIVERGED_DUE_TO LINESEARCH
# adding line_search = none

# getting DIVERGED_DTOL
# added debug statements
# shows that disp variables similar magnitude but pp is very small 1e-20
# added automatic scaling but it might be IC problem

# okay so one issue might be that the porosisy is starting at zero
# which it should be at 0.1
# noticed some other things  had use_displaced_mesh and turned them on
# i get a warning that says theres a negative in the jacobian
# and an aux variable caluclation issue
# i think this is due to negative porosity or some such
# which is caused by element distortrion in the mesh
# mesh refinement or adaptivity might help

# trying adding more nodes
# Looks like some possible issues:
# nodes deforming TOO much
# or negative porosity
# i looked and at timestep 0
# porosity is 0 which seems bad
# BUT thats how its supposed to be and
# you can say start_time = -something
# to get rid of that
# so porosity should be fine

# nonlinear convergence is very slow/ nonexistant
# tried FDP
# shows three negative values in the jacobian
# again issues due to mesh distortrion
# got rid of two poles DirichletBCs
# now only 1 place has negative Jacobian
# so its related to BCS

# tried reducing magnitude of BodyForcing but that
# doesn't make a difference
# tried use_displaced_mesh= false and that didn't help
#
# increasing the size helped a bit to get to 1 negative
# decreasing the speed by which the load is applied didn't seem to help
# tried addint PorousFlowCapillaryPressureConst
# tried commenting out pp BC

# changed nx from 10 to 40
# jacobian was LESS negative (still negative but less)
# resididual of porepressure still very low compared to disp
# and not even close to the same order
# increased to 40 30 20 nodes and it got to -0.1 jacobian

# added more nodes adnn PorousFlowOutflowBC
# maybe helped but now linear solves do not converge



[Mesh]
  displacements = 'disp_x disp_y disp_z'
  [the_mesh]
    type = GeneratedMeshGenerator
    dim = 3
    nx = 40
    ny = 40
    nz = 40
    xmin = 0
    xmax = 100
    ymin = 0
    ymax = 100
    zmin = 0
    zmax = 100
  []
  [pole1]
    type = ExtraNodesetGenerator
    input = the_mesh
    new_boundary = 111
    coord = '50 50 50'
  []
  [pole2]
    type = ExtraNodesetGenerator
    input = pole1
    new_boundary = 110
    coord = '50 60 50'
  []
  [pole3]
    type = ExtraNodesetGenerator
    input = pole2
    new_boundary = 100
    coord = '50 50 60'
  []
[]

[GlobalParams]
  displacements = 'disp_x disp_y disp_z'
  PorousFlowDictator = dictator
  block = 0
[]

[UserObjects]
  [dictator]
    type = PorousFlowDictator
    porous_flow_vars = 'porepressure disp_x disp_y disp_z'
    number_fluid_phases = 1
    number_fluid_components = 1
  []
  [pc]
  type = PorousFlowCapillaryPressureConst
  pc = 0
  []
[]

[AuxVariables]
  [porosity]
    order = FIRST
    family = MONOMIAL
  []
  [perm_x]
     order = FIRST
     family = MONOMIAL
   []
   [perm_y]
     order = FIRST
     family = MONOMIAL
   []
   [perm_z]
     order = FIRST
     family = MONOMIAL
   []
  #  [g_aux]
  #    order = FIRST
  #    family = LAGRANGE
  # []
[]

 [AuxKernels]
   [poro]
     type = PorousFlowPropertyAux
     property = porosity
     variable = porosity
   []
   [perm_x]
     type = PorousFlowPropertyAux
     property = permeability
     variable = perm_x
     row = 0
     column = 0
   []
   [perm_y]
     type = PorousFlowPropertyAux
     property = permeability
     variable = perm_y
     row = 1
     column = 1
   []
   [perm_z]
     type = PorousFlowPropertyAux
     property = permeability
     variable = perm_z
     row = 2
     column = 2
   []
[]

[Variables]
  [disp_x]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0
    scaling = 1e-4
  []
  [disp_y]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0
    scaling = 1e-4
  []
  [disp_z]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0
    scaling = 1e-4
  []
  [porepressure]
    scaling = 1e10
  []

[]

[BCs]
  [ppbc]
    type = PorousFlowPiecewiseLinearSink
    variable = porepressure
    boundary = 'left right top bottom back front'
    pt_vals = '1e-9 1e9'
    multipliers = '1e-9 1e9'
    flux_function = 1
    PT_shift = 1e7
    use_displaced_mesh = true 
    use_mobility = true
    use_relperm = true
    fluid_phase = 0
  []
  [polex]
    type = DirichletBC
    variable = disp_x
    value = 0
    boundary = 111
    use_displaced_mesh = true
  []
  [poley]
    type = DirichletBC
    variable = disp_z
    value = 0
    boundary = 111
    use_displaced_mesh = true
  []
  [polez]
    type = DirichletBC
    variable = disp_z
    value = 0
    boundary = 111
    use_displaced_mesh = true
  []
  # [pole2z]
  #   type = DirichletBC
  #   variable = disp_z
  #   value = 0
  #   boundary = 110
  #   use_displaced_mesh = true
  # []
  # [pole2x]
  #   type = DirichletBC
  #   variable = disp_x
  #   value = 0
  #   boundary = 110
  #   use_displaced_mesh = true
  # []
  # [pole3y]
  #   type = DirichletBC
  #   variable = disp_x
  #   value = 0
  #   boundary = 100
  #   use_displaced_mesh = true
  # []
[]

[ICs]
  [pp_ic]
    type = ConstantIC
    variable = porepressure
    value = 1e7
  []
  [poro_ic]
    type = ConstantIC
    variable = porosity
    value = 0.1
  []
[]

[Functions]
  [gravf]
    type = ParsedFunction
    expression = -.05*cos(t/500)
  []
  [test]
    type = ConstantFunction
    value = 1
  []
[]

[FluidProperties]
  [the_simple_fluid]
    type = SimpleFluidProperties
    bulk_modulus = 2E9
  []
[]

[Kernels]
  [grad_stress_x]
    type = StressDivergenceTensors
    variable = disp_x
    component = 0
    use_displaced_mesh = true
  []
  [grad_stress_y]
    type = StressDivergenceTensors
    variable = disp_y
    component = 1
    use_displaced_mesh = true
  []
  [grad_stress_z]
    type = StressDivergenceTensors
    variable = disp_z
    component = 2
    use_displaced_mesh = true
  []
  [poro_x]
    type = PorousFlowEffectiveStressCoupling
    biot_coefficient = 0.6
    variable = disp_x
    component = 0
    use_displaced_mesh = true
  []
  [poro_y]
    type = PorousFlowEffectiveStressCoupling
    biot_coefficient = 0.6
    variable = disp_y
    component = 1
    use_displaced_mesh = true
  []
  [poro_z]
    type = PorousFlowEffectiveStressCoupling
    biot_coefficient = 0.6
    component = 2
    variable = disp_z
    use_displaced_mesh = true
  []
  [poro_vol_exp]
    type = PorousFlowMassVolumetricExpansion
    variable = porepressure
    fluid_component = 0
  []
  [mass0]
    type = PorousFlowMassTimeDerivative
    fluid_component = 0
    variable = porepressure
  []
  [flux]
    type = PorousFlowAdvectiveFlux
    variable = porepressure
    gravity = '0 0 0'
    fluid_component = 0
  []
  [gravy]
    type = BodyForce
    variable = disp_x
    function = gravf
    use_displaced_mesh = true
  []
[]

[Materials]
  [temperature]
    type = PorousFlowTemperature
  []
  [elasticity_tensor]
    type = ComputeIsotropicElasticityTensor
    bulk_modulus = 1E5 # drained bulk modulus
    poissons_ratio = 0.25
    use_displaced_mesh = true
  []
  [strain]
    type = ComputeFiniteStrain # changed from ComputeSmallStrain
  []
  [stress]
    type = ComputeFiniteStrainElasticStress #changed from ComputeLinearElasticStress

  []
  [eff_fluid_pressure]
    type = PorousFlowEffectiveFluidPressure
    use_displaced_mesh = true
  []
  [vol_strain]
    type = PorousFlowVolumetricStrain
    use_displaced_mesh = true
  []
  [ppss]
    type = PorousFlow1PhaseFullySaturated
    porepressure = porepressure
  []
  [massfrac]
    type = PorousFlowMassFraction
  []
  [simple_fluid]
    type = PorousFlowSingleComponentFluid
    fp = the_simple_fluid
    phase = 0
    use_displaced_mesh = true
  []
  [porosity]
    type = PorousFlowPorosity
    fluid = true
    mechanical = true
    porosity_zero = 0.1
    biot_coefficient = 0.6
    solid_bulk = 1E5
    use_displaced_mesh = true
    ensure_positive = true
  []
  [permeability]
    type = PorousFlowPermeabilityKozenyCarman
    poroperm_function = kozeny_carman_phi0
    k0 = 10e-12
    phi0 = 0.1
    m = 2
    n = 2
    use_displaced_mesh = true
  []

  [relperm]
    type = PorousFlowRelativePermeabilityCorey
    n = 0 # unimportant in this fully-saturated situation
    phase = 0
    use_displaced_mesh = true
  []
  [density]
    type = PorousFlowTotalGravitationalDensityFullySaturatedFromPorosity
    rho_s = 2400
    use_displaced_mesh = true
  []

[]

[Dampers]
  [./disp_x_damp]
    type = ElementJacobianDamper
    displacements = 'disp_x disp_y disp_z'
  [../]
[]



[Postprocessors]
  [pp]
    type = PointValue
    point = '0.5 0.5 0.5'
    variable = porepressure
    execute_on = "INITIAL TIMESTEP_END"
  []
  [poro]
    type = ElementExtremeValue
    #point = '0.5 0.5 0.5'
    variable = porosity
    execute_on = "INITIAL TIMESTEP_END"
  []
  [permx]
    type = PointValue
    point = '0.5 0.5 0.5'
    variable = perm_x
    execute_on = "INITIAL TIMESTEP_END"
  []
  [dispx]
    type = PointValue
    point = '0.5 0.5 0.5'
    variable = disp_x
    execute_on = "INITIAL TIMESTEP_END"
  []
[]

[Preconditioning]
  active = mumps
  [simp]
    type = SMP
    full = true
  []
  [mumps]
    type = SMP
    full = true
    petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
    petsc_options_value = ' lu       mumps'
  []
  [FDP]
    type = FDP
  []
[]

[Executioner]
  type = Transient
  solve_type = Newton
  end_time = 50
  dt = 1e-8
  l_max_its = 5000
  line_search = none

  # petsc_options = '-pc_svd_monitor'
  # petsc_options_iname = '-pc_type'
  # petsc_options_value = 'svd'
  # line_search = 'none'
  # [./TimeStepper]
  #   type = IterationAdaptiveDT
  #   dt = 0.1
  # []
[]


[Outputs]
  console = true
  csv = true
  exodus = true
  execute_on = 'initial timestep_end failed'
[]

[Debug]
  show_var_residual_norms = true
  show_top_residuals = 3
[]
