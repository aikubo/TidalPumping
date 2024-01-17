# A confined aquifer is fully saturated with water
# Earth tides apply strain to the aquifer and the resulting porepressure changes are recorded
#
# To replicate standard poroelasticity exactly:
# (1) the PorousFlowBasicTHM Action is used;
# (2) multiply_by_density = false;
# (3) PorousFlowConstantBiotModulus is used
[Mesh]
  type = GeneratedMesh
  dim = 3
  nx = 1
  ny = 1
  nz = 1
  xmin = 0
  xmax = 1
  ymin = 0
  ymax = 1
  zmin = 0
  zmax = 1
[]

[GlobalParams]
  displacements = 'disp_x disp_y disp_z'
  PorousFlowDictator = dictator
  block = 0
  biot_coefficient = 0.6
  multiply_by_density = false
[]

[Variables]
  [disp_x]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0
  []
  [disp_y]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0
  []
  [disp_z]
    order = FIRST
    family = LAGRANGE
    initial_condition = 0
  []
  [porepressure]

  []
[]

  [outflow]
    type = DirichletBC
    boundary = 'left right top bottom back front'
    variable = porepressure
    value = 10
  []

  [polex]
    type = DirichletBC
    variable = disp_x
    value = 0
    boundary = 111
  []
  [poley]
    type = DirichletBC
    variable = disp_z
    value = 0
    boundary = 111
  []
  [polez]
    type = DirichletBC
    variable = disp_z
    value = 0
    boundary = 111
  []
  [pole2z]
    type = DirichletBC
    variable = disp_z
    value = 0
    boundary = 110
  []
  [pole2x]
    type = DirichletBC
    variable = disp_x
    value = 0
    boundary = 110
  []
  [pole3y]
    type = DirichletBC
    variable = disp_y
    value = 0
    boundary = 100
  []
[]


[Functions]
  [gravf]
    type = ParsedFunction
    expression = -10*cos(t/5)
  []
[]

[FluidProperties]
  [the_simple_fluid]
    type = SimpleFluidProperties
    bulk_modulus = 2E9
  []
[]

[PorousFlowFullySaturated]
  porepressure = porepressure
  coupling_type = HydroMechanical
  displacements = 'disp_x disp_y disp_z'
  fp = the_simple_fluid
  biot_coefficient = 0.6
  gravity = ' 0 0 0'
  #stabilization = KT
[]

[Kernels]
  [gravy]
    type = BodyForce
    variable = disp_x
    function = gravf
    use_displaced_mesh = true
  []
[]

[Materials]
  [elasticity_tensor]
    type = ComputeIsotropicElasticityTensor
    bulk_modulus = 10.0E9 # drained bulk modulus
    poissons_ratio = 0.25
  []
  [strain]
    type = ComputeFiniteStrain
  []
  [stress]
    type = ComputeFiniteStrainElasticStress
  [porosity]
    type = PorousFlowPorosity
    fluid = true
    mechanical = true
    porosity_zero = 0.1
    biot_coefficient = 0.6
    solid_bulk = 1E9
  []
  [biot_modulus]
    type = PorousFlowConstantBiotModulus
    biot_coefficient = 0.6
  []
  [permeability]
    type = PorousFlowPermeabilityKozenyCarman
    poroperm_function = kozeny_carman_phi0
    k0 = 10e-12
    phi0 = 0.1
    m = 2
    n = 7
  []
  [relperm]
    type = PorousFlowRelativePermeabilityCorey
    n = 0 # unimportant in this fully-saturated situation
    phase = 0
  []
  [density]
    type = PorousFlowTotalGravitationalDensityFullySaturatedFromPorosity
    rho_s = 2400
  []
[]

[Postprocessors]
  [pp]
    type = PointValue
    point = '0.5 0.5 0.5'
    variable = porepressure
  []
[]

[Preconditioning]
  [smp]
    type = SMP
    full = true
  []
[]

[Executioner]
  type = Transient
  solve_type = Newton
  dt = 0.01
  end_time = 2
[]

[Outputs]
  console = true
  csv = true
  exodus = true
[]
