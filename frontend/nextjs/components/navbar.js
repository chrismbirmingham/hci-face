export default function Navbar() {
    return (
        <header className='container flex justify-between h-12 items-center ml-4'>
            <nav>
                <ul className='flex gap-6 font-medium items-center'>
                    <li><a className="text-2xl" href='/'>Home</a></li>
                    <li><a href='/transcription'>Transcribe</a></li>
                    <li><a href='/puppeteering'>Puppeteer</a></li>
                    <li><a href='/multimodal_interaction'>Interact</a></li>
                </ul>
            </nav>
        </header>
    )
}