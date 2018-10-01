if ! haskey(ENV, "BULLET3")
  run(`./install-bullet3.sh`)
  println("Installed bullet3 at ", joinpath(ENV["HOME"], "bullet3"))
else
  println("Found bullet3 at $(ENV["BULLET3"])")
end
