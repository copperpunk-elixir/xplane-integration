defmodule XplaneIntegration.MixProject do
  use Mix.Project

  @version "0.1.4"
  @source_url "https://github.com/copperpunk-elixir/xplane-integration"

  def project do
    [
      app: :xplane_integration,
      version: @version,
      elixir: "~> 1.12",
      description: description(),
      package: package(),
      source_url: @source_url,
      docs: docs(),
      start_permanent: Mix.env() == :prod,
      deps: deps()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp description do
    "Receive/Send data messages to X-Plane flight simulation. Documentation is INCOMPLETE!"
  end

  defp package do
    %{
      licenses: ["GPL-3.0"],
      links: %{"Github" => @source_url}
    }
  end

  defp docs do
    [
      extras: ["README.md"],
      main: "readme",
      source_ref: "v#{@version}",
      source_url: @source_url
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:ex_doc, "~> 0.24", only: :dev, runtime: false},
      # git: "https://github.com/copperpunk-elixir/via-utils.git", tag: "v0.1.4-alpha"}
      {:via_utils,
       path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/via-utils", override: true},
      {:ubx_interpreter,
       path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/ubx-interpreter"},
      {:via_simulation, path: "/home/ubuntu/Documents/Github/cp-elixir/libraries/via-simulation"}

      # {:ubx_interpreter, "~> 0.1.1"}
      # {:dep_from_hexpm, "~> 0.3.0"},
      # {:dep_from_git, git: "https://github.com/elixir-lang/my_dep.git", tag: "0.1.0"}
    ]
  end
end
