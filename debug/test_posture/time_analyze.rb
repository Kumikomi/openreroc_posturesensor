#!/usr/bin/ruby
argv = ARGV[0].to_s

i = 0;

fd = open(argv,"r")
fo = open("result/#{argv.delete("log/"+"/.txt")}.csv","w")
fo.puts "x,y,z,t"
while l = fd.gets
	data = l.chomp.split(":")
	if i > 3
		i = 0;
	end
	case i
		when 0
          a = data[1]
		when 1
			b = data[1]
                when 2
                        c = data[1]
                when 3 
                        all = data[1]
        end
	if i == 3
          fo.puts "#{a},#{b},#{c},#{all}"
	end
	i = i + 1
end
