export default function Navbar() {
    return (
        <header className='max-w-full container flex justify-between h-12 items-center bg-white border border-black'>
            <nav>
                <ul className='flex  ml-4 gap-6 font-medium items-center'>
                    <li><a className="text-2xl" href='/'>Home</a></li>
                    <li><a href='/transcription'>Transcribe</a></li>
                    <li><a href='/puppeteering'>Puppeteer</a></li>
                    <li><a href='/multimodal_interaction'>Interact</a></li>
                </ul>
            </nav>
        </header>
    )
}