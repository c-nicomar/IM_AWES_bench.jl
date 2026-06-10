using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Test
using IM_AWES_bench

@testset "Basic arithmetic" begin
    @test 1+1 == 2
end